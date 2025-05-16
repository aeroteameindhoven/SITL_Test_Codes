from dronekit import connect
from pymavlink import mavutil
import time
import math
import matplotlib.pyplot as plt

# --- PID Controller ---
class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
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

# --- Targets ---
target_airspeed = 18.0
target_altitude = 50.0
target_heading_deg = 135.0
target_heading_rad = math.radians(target_heading_deg)

# --- Tuned Controllers ---
airspeed_pid = PID(kp=0.012, ki=0.004, kd=0.12, integral_limit=4.0)
altitude_pid = PID(kp=0.07, ki=0.018, kd=0.12, integral_limit=6.0)
heading_to_roll_pid = PID(kp=2.2, ki=0.0, kd=0.25, integral_limit=0.3)

# --- Timing ---
start_time = time.time()
last_time = start_time
duration = 90  # seconds

# --- Logging ---
log_time = []
log_airspeed = []
log_throttle = []
log_altitude = []
log_heading = []
log_roll_cmd = []

# --- Initial Throttle ---
initial_throttle = 0.5
if hasattr(vehicle, 'channels') and '3' in vehicle.channels:
    try:
        pwm = int(vehicle.channels['3'])
        initial_throttle = (pwm - 1000) / 1000
    except:
        pass

# --- Warmup ---
print("Warming up throttle...")
q_init = euler_to_quaternion(0.0, 0.0, 0.0)
for _ in range(20):
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

# --- Initialize throttle ---
throttle_cmd = initial_throttle
airspeed_error = target_airspeed - (vehicle.airspeed or 0.0)
if airspeed_pid.ki > 0:
    airspeed_pid.integral = (initial_throttle - airspeed_pid.kp * airspeed_error) / airspeed_pid.ki

print("Running 90s airspeed + altitude + heading hold...")
while True:
    now = time.time()
    dt = now - last_time
    elapsed = now - start_time
    last_time = now

    if elapsed > duration:
        break

    # --- Telemetry
    current_airspeed = vehicle.airspeed or 0.0
    current_altitude = vehicle.location.global_relative_frame.alt or 0.0
    heading_deg = vehicle.heading or 0.0
    heading_rad = math.radians(heading_deg)

    # --- Error Computation
    airspeed_error = target_airspeed - current_airspeed
    altitude_error = target_altitude - current_altitude
    heading_error = ((target_heading_rad - heading_rad + math.pi) % (2 * math.pi)) - math.pi

    # --- Airspeed PID (throttle)
    delta_throttle = airspeed_pid.update(airspeed_error, dt)
    throttle_cmd += delta_throttle

    if log_throttle:
        prev = log_throttle[-1]
        alpha = 0.2
        throttle_cmd = alpha * throttle_cmd + (1 - alpha) * prev
        max_delta = 0.03
        throttle_cmd = max(min(throttle_cmd, prev + max_delta), prev - max_delta)

    throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

    # --- Altitude PID (pitch)
    pitch_cmd = altitude_pid.update(altitude_error, dt)
    pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))

    # --- Heading PID (roll in degrees, then to radians)
    roll_cmd_deg = heading_to_roll_pid.update(heading_error, dt)
    roll_cmd_deg = max(min(roll_cmd_deg, 25), -25)

    if log_roll_cmd:
        roll_cmd_deg = 0.2 * roll_cmd_deg + 0.8 * log_roll_cmd[-1]  # smoothing

    roll_cmd_rad = math.radians(roll_cmd_deg)
    if not math.isfinite(roll_cmd_rad):
        roll_cmd_rad = 0.0  # fail-safe

    # --- Command UAV
    q = euler_to_quaternion(roll_cmd_rad, pitch_cmd, 0.0)
    vehicle._master.mav.set_attitude_target_send(
        int(elapsed * 1000),
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,
        q,
        0.0, 0.0, 0.0,
        throttle_cmd
    )

    # --- Logging
    log_time.append(elapsed)
    log_airspeed.append(current_airspeed)
    log_throttle.append(throttle_cmd)
    log_altitude.append(current_altitude)
    log_heading.append(heading_deg)
    log_roll_cmd.append(roll_cmd_deg)

    print(f"[{int(elapsed)}s] AS: {current_airspeed:.2f} | ALT: {current_altitude:.2f} | HDG: {heading_deg:.1f} | ROLL: {roll_cmd_deg:.1f} | THR: {throttle_cmd:.3f}")
    time.sleep(0.01)

# --- Plotting ---
plt.figure(figsize=(12, 10))

plt.subplot(4, 1, 1)
plt.plot(log_time, log_airspeed, label="Airspeed")
plt.plot(log_time, [target_airspeed]*len(log_time), '--', label="Target Airspeed")
plt.ylabel("Airspeed (m/s)")
plt.grid(True)
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(log_time, log_altitude, label="Altitude")
plt.plot(log_time, [target_altitude]*len(log_time), '--', label="Target Altitude")
plt.ylabel("Altitude (m)")
plt.grid(True)
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(log_time, log_heading, label="Heading")
plt.plot(log_time, [target_heading_deg]*len(log_time), '--', label="Target Heading")
plt.ylabel("Heading (deg)")
plt.grid(True)
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(log_time, log_roll_cmd, label="Roll Cmd (deg)")
plt.ylabel("Roll Cmd (deg)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
print("✅ Done.")
