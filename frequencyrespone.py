from dronekit import connect
from pymavlink import mavutil
import time
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from scipy.fft import fft, fftfreq
from scipy.signal import detrend
from numpy import unwrap, angle
import csv


# --- PID Controller ---
class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# --- Quaternion Helper ---
def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ]

# --- Connect to vehicle ---
print("Connecting to vehicle...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Connected!")

# --- Target Params ---
target_airspeed = 17.0  # m/s
target_altitude = 48.0  # m

# --- PID Controllers ---
airspeed_pid = PID(kp=0.02, ki=0.006, kd=0.08)
altitude_pid = PID(kp=0.035, ki=0.012, kd=0.16, integral_limit=5.0)


# --- Timing ---
start_time = time.time()
last_time = start_time

# --- Logging ---
log_time = []
log_airspeed = []
log_throttle = []
log_altitude = []

# --- Estimate Initial Throttle ---
initial_throttle = 0.5
if hasattr(vehicle, 'channels') and '3' in vehicle.channels:
    try:
        pwm = int(vehicle.channels['3'])
        initial_throttle = (pwm - 1000) / 1000
    except:
        pass

# --- Prime throttle with attitude hold ---
q_init = euler_to_quaternion(0.0, 0.0, 0.0)
for _ in range(20):  # ~200ms warmup
    vehicle._master.mav.set_attitude_target_send(
        0,
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,
        q_init,
        0.0, 0.0, 0.0,
        initial_throttle
    )
    time.sleep(0.01)

# --- Initialize PID integral to match initial throttle ---
current_airspeed = vehicle.airspeed or 0.0
airspeed_error = target_airspeed - current_airspeed
if airspeed_pid.ki > 0:
    airspeed_pid.integral = (initial_throttle - airspeed_pid.kp * airspeed_error) / airspeed_pid.ki

# --- Main Loop ---
print("Running airspeed test with altitude hold...")
duration = 90  # seconds

while True:
    now = time.time()
    dt = now - last_time
    elapsed = now - start_time
    last_time = now

    if elapsed > duration:
        break

    current_airspeed = vehicle.airspeed or 0.0
    current_altitude = vehicle.location.global_relative_frame.alt or 0.0

    airspeed_error = target_airspeed - current_airspeed
    altitude_error = target_altitude - current_altitude

    # --- PID Control ---
    throttle_cmd = airspeed_pid.update(airspeed_error, dt)
    throttle_cmd = max(0.0, min(throttle_cmd, 1.0))
    pitch_cmd = altitude_pid.update(altitude_error, dt)
    pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))
    

    # --- Attitude Command ---
    q = euler_to_quaternion(0.0, pitch_cmd, 0.0)
    vehicle._master.mav.set_attitude_target_send(
        int(elapsed * 1000),
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,
        q,
        0.0, 0.0, 0.0,
        throttle_cmd
    )

    # --- Logging ---
    log_time.append(elapsed)
    log_airspeed.append(current_airspeed)
    log_throttle.append(throttle_cmd)
    log_altitude.append(current_altitude)

    print(f"[{int(elapsed * 1000)} ms] AS: {current_airspeed:.2f} m/s | Alt: {current_altitude:.2f} m | Throttle: {throttle_cmd:.3f}")
    time.sleep(0.01)

# --- Plotting Time-Series ---
plt.figure(figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(log_time, log_airspeed, label="Airspeed")
plt.plot(log_time, [target_airspeed] * len(log_time), '--', label="Target AS")
plt.ylabel("Airspeed (m/s)")
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(log_time, log_altitude, label="Altitude")
plt.plot(log_time, [target_altitude] * len(log_time), '--', label="Target Alt")
plt.ylabel("Altitude (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# --- FFT Setup ---
dt = 0.01  # Your sampling time (100 Hz)
T = log_time[-1]
N = int(T / dt)
uniform_time = np.linspace(0, T, N)

# Interpolate throttle and airspeed to uniform time grid
interp_throttle = interp1d(log_time, log_throttle, kind='linear', fill_value='extrapolate')
interp_airspeed = interp1d(log_time, log_airspeed, kind='linear', fill_value='extrapolate')
throttle_u = interp_throttle(uniform_time)
airspeed_y = interp_airspeed(uniform_time)

# FFT and frequency response
U = fft(throttle_u)
Y = fft(airspeed_y)
freqs = fftfreq(N, dt)

# Positive frequencies only
idx = freqs > 0
freqs = freqs[idx]
H = Y[idx] / U[idx]
gain_db = 20 * np.log10(np.abs(H))
phase_deg = np.angle(H, deg=True)

# --- Plot Bode Plot ---
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.semilogx(freqs, gain_db)
plt.ylabel("Gain (dB)")
plt.title("Bode Plot: Throttle → Airspeed")
plt.grid(True)

plt.subplot(2, 1, 2)
plt.semilogx(freqs, phase_deg)
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (deg)")
plt.grid(True)

plt.tight_layout()
plt.show()

log_filename = "airspeed_altitude_log.csv"
with open(log_filename, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", "airspeed", "altitude", "throttle"])
    for t, a, alt, thr in zip(log_time, log_airspeed, log_altitude, log_throttle):
        writer.writerow([t, a, alt, thr])

print(f"✅ Logged data to {log_filename}")