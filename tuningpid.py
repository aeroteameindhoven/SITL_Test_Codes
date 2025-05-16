from dronekit import connect
from pymavlink import mavutil
import time
import math
import matplotlib.pyplot as plt

# --- PID Controller (ki and kd = 0 for Z-N method) ---
class PID:
    def __init__(self, kp, ki=0.3552, kd=0.074, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        self.integral += error * dt
        if self.integral_limit:
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
target_altitude = 50  # meters
altitude_pid = PID(kp=0.0888)  # Start small, set ki=kd=0 for Ziegler-Nichols
airspeed_throttle = 0.5     # Constant throttle for test (or set a value that holds 20 m/s)

# --- Timing ---
start_time = time.time()
last_time = start_time

# --- Logging for plotting ---
log_time = []
log_altitude = []
log_pitch_deg = []

# --- Main loop ---
print("Running Ziegler–Nichols test...")
duration = 120  # seconds

while True:
    now = time.time()
    dt = now - last_time
    elapsed = now - start_time
    last_time = now

    if elapsed > duration:
        break

    # Get telemetry
    current_altitude = vehicle.location.global_relative_frame.alt or 0.0
    altitude_error = target_altitude - current_altitude

    # PID pitch
    pitch_cmd = altitude_pid.update(altitude_error, dt)
    pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))
    pitch_deg = math.degrees(pitch_cmd)

    # Convert to quaternion and send command
    q = euler_to_quaternion(0.0, pitch_cmd, 0.0)
    vehicle._master.mav.set_attitude_target_send(
        int(elapsed * 1000),
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,
        q,
        0.0, 0.0, 0.0,
        airspeed_throttle
    )

    # Log
    log_time.append(elapsed)
    log_altitude.append(current_altitude)
    log_pitch_deg.append(pitch_deg)

    print(f"[{int(elapsed*1000)} ms] Alt: {current_altitude:.2f} | Pitch: {pitch_deg:.2f}°")

    time.sleep(0.01)

# --- Plotting the Z-N response ---
plt.figure(figsize=(10, 6))
plt.plot(log_time, log_altitude, label="Altitude")
plt.plot(log_time, [target_altitude]*len(log_time), '--', label="Target")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Ziegler–Nichols Tuning: Altitude Response")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
