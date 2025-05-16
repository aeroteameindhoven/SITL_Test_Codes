import numpy as np
import time
from pymavlink import mavutil
import math
import matplotlib.pyplot as plt

# === Connect to SITL ===
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
master.wait_heartbeat()
print("✅ Connected to UAV")

boot_time = time.time()

# Request RAW_IMU messages at 50Hz
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    27,        # RAW_IMU msg ID
    20000,     # 20ms = 50Hz
    0, 0, 0, 0, 0
)

# === PID Constants ===
Kp_throttle = 0.08
Ki_throttle = 0.025
Kd_throttle = 0.003

Kp_pitch = 0.006
Ki_pitch = 0.05
Kd_pitch = 0.004

Kp_roll = 0.004
Ki_roll = 0.0004
Kd_roll = 0.0006

# === Setpoints ===
ax_sp = 2.0       # Forward acceleration
ay_sp = 0.0       # No sideways movement
az_sp = -9.6      # Hover descent

# === Time Step ===
DT = 0.01

# === Helper: Euler to Quaternion ===
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

# === Send Attitude Target ===
def send_attitude_cmd(roll_rad, pitch_rad, throttle=0.5):
    q = euler_to_quaternion(roll_rad, pitch_rad, 0.0)
    now_ms = int((time.time() - boot_time) * 1000)
    master.mav.set_attitude_target_send(
        now_ms,
        master.target_system,
        master.target_component,
        0b00000111,
        q,
        0, 0, 0,
        throttle
    )

# === Get IMU Accelerations ===
def get_accels():
    msg = master.recv_match(type="RAW_IMU", blocking=True)
    if msg:
        return (
            msg.xacc * 9.80665 / 1000,
            msg.yacc * 9.80665 / 1000,
            msg.zacc * 9.80665 / 1000
        )
    return 0.0, 0.0, -9.8

# === Logs ===
time_log = []
ax_log, ay_log, az_log = [], [], []
pitch_log, roll_log, throttle_log = [], [], []

# === PID State ===
integral_pitch = integral_roll = integral_throttle = 0.0
prev_error_pitch = prev_error_roll = prev_error_throttle = 0.0

# === Throttle Warmup ===
print("⚙️ Warming up throttle before control loop...")
q_init = euler_to_quaternion(0.0, 0.0, 0.0)
for _ in range(20):  # 200ms
    now_ms = int((time.time() - boot_time) * 1000)
    master.mav.set_attitude_target_send(
        now_ms,
        master.target_system,
        master.target_component,
        0b00000111,
        q_init,
        0.0, 0.0, 0.0,
        0.5
    )
    time.sleep(0.01)

# === Control Loop ===
start_time = time.time()
while time.time() - start_time < 60:
    ax_now, ay_now, az_now = get_accels()

    # Error signals
    err_pitch = az_sp - az_now
    err_roll = ay_sp - ay_now
    err_throttle = ax_sp - ax_now

    # PID: Pitch (az)
    integral_pitch += err_pitch * DT
    derivative_pitch = (err_pitch - prev_error_pitch) / DT
    pitch_cmd = Kp_pitch * err_pitch + Ki_pitch * integral_pitch + Kd_pitch * derivative_pitch
    pitch_cmd = np.clip(pitch_cmd, -math.radians(45), math.radians(45))
    prev_error_pitch = err_pitch

    # PID: Roll (ay)
    integral_roll += err_roll * DT
    derivative_roll = (err_roll - prev_error_roll) / DT
    roll_cmd = Kp_roll * err_roll + Ki_roll * integral_roll + Kd_roll * derivative_roll
    roll_cmd = np.clip(roll_cmd, -math.radians(45), math.radians(45))
    prev_error_roll = err_roll

    # PID: Throttle (ax)
    integral_throttle += err_throttle * DT
    derivative_throttle = (err_throttle - prev_error_throttle) / DT
    throttle_cmd = Kp_throttle * err_throttle + Ki_throttle * integral_throttle + Kd_throttle * derivative_throttle
    throttle_cmd = np.clip(0.5 + throttle_cmd, 0.0, 1.0)
    prev_error_throttle = err_throttle

    # Send command
    send_attitude_cmd(roll_cmd, pitch_cmd, throttle_cmd)

    # Logging
    t_now = time.time() - start_time
    time_log.append(t_now)
    ax_log.append(ax_now)
    ay_log.append(ay_now)
    az_log.append(az_now)
    pitch_log.append(math.degrees(pitch_cmd))
    roll_log.append(math.degrees(roll_cmd))
    throttle_log.append(throttle_cmd)

    print(f"[{t_now:.2f}s] ax: {ax_now:.2f} | ay: {ay_now:.3f} | az: {az_now:.2f} | pitch: {math.degrees(pitch_cmd):.2f}° | roll: {math.degrees(roll_cmd):.2f}° | thr: {throttle_cmd:.2f}")
    time.sleep(DT)

# === Plotting ===
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time_log, ax_log, label="ax")
plt.plot(time_log, [ax_sp]*len(time_log), 'r--', label="ax_sp")
plt.ylabel("ax (m/s²)")
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time_log, ay_log, label="ay")
plt.plot(time_log, [ay_sp]*len(time_log), 'r--', label="ay_sp")
plt.ylabel("ay (m/s²)")
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time_log, az_log, label="az")
plt.plot(time_log, [az_sp]*len(time_log), 'r--', label="az_sp")
plt.xlabel("Time (s)")
plt.ylabel("az (m/s²)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

# === Control Output Plots ===
plt.figure(figsize=(12, 6))
plt.subplot(3, 1, 1)
plt.plot(time_log, pitch_log, label="Pitch (°)")
plt.grid()
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_log, roll_log, label="Roll (°)")
plt.grid()
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_log, throttle_log, label="Throttle")
plt.xlabel("Time (s)")
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
