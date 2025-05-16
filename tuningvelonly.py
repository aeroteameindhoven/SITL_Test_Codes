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

# --- Connect to Vehicle ---
print("Connecting to vehicle...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Connected!")

# --- Target Airspeed ---
target_airspeed = 18.0

# --- PID Gains (adjust as needed) ---
airspeed_pid = PID(kp=0.012, ki=0.004, kd=0.12, integral_limit=4.0)

# --- Duration and Timing ---
duration = 90  # seconds
start_time = time.time()
last_time = start_time

# --- Logging ---
log_time = []
log_airspeed = []
log_throttle = []

# --- Initial throttle ---
throttle_cmd = 0.5
q_level = euler_to_quaternion(0.0, 0.0, 0.0)

# --- Warmup ---
print("Warming up...")
for _ in range(20):
    vehicle._master.mav.set_attitude_target_send(
        0,
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,
        q_level,
        0.0, 0.0, 0.0,
        throttle_cmd
    )
    time.sleep(1/15)

print(f"Running airspeed hold for {duration} seconds...")
while True:
    now = time.time()
    dt = now - last_time
    elapsed = now - start_time
    last_time = now

    if elapsed > duration:
        break

    # Read airspeed
    current_airspeed = vehicle.airspeed or 0.0

    # Filter for smoothness
    if log_airspeed:
        current_airspeed = 0.85 * log_airspeed[-1] + 0.15 * current_airspeed

    # PID update
    airspeed_error = target_airspeed - current_airspeed
    delta_throttle = airspeed_pid.update(airspeed_error, dt)
    throttle_cmd += delta_throttle

    # Throttle smoothing and limits
    if log_throttle:
        prev = log_throttle[-1]
        throttle_cmd = 0.2 * throttle_cmd + 0.8 * prev
        throttle_cmd = max(min(throttle_cmd, prev + 0.02), prev - 0.02)

    throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

    # Send attitude + throttle (level flight)
    vehicle._master.mav.set_attitude_target_send(
        int(elapsed * 1000),
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,
        q_level,
        0.0, 0.0, 0.0,
        throttle_cmd
    )

    # Log data
    log_time.append(elapsed)
    log_airspeed.append(current_airspeed)
    log_throttle.append(throttle_cmd)

    print(f"[{int(elapsed)}s] AS: {current_airspeed:.2f} | THR: {throttle_cmd:.3f}")
    time.sleep(0.01)

# --- Plot Results ---
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(log_time, log_airspeed, label="Airspeed")
plt.plot(log_time, [target_airspeed]*len(log_time), '--', label="Target")
plt.ylabel("Airspeed (m/s)")
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(log_time, log_throttle, label="Throttle")
plt.ylabel("Throttle (0-1)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

vehicle.close()
print("✅ Airspeed-only PID controller finished.")
