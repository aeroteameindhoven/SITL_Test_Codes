import numpy as np
import time
from pymavlink import mavutil
import math
import matplotlib.pyplot as plt

# === MAVLink Setup ===
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
master.wait_heartbeat()
print("✅ Connected to UAV")

boot_time = time.time()

# Request RAW_IMU at 50Hz
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 27, 20000,
    0, 0, 0, 0, 0
)

# === PID Constants ===
# ax (via throttle)
Kp_throttle = 0.018
Ki_throttle = 0.030
Kd_throttle = 0.0035

# az (via pitch)
Kp_pitch = 0.0042
Ki_pitch = 0.045
Kd_pitch = 0.0030

# ay (via roll)
Kp_roll = 0.0045
Ki_roll = 0.0003
Kd_roll = 0.0005

# === Setpoints ===
ax_sp = 2
ay_sp = 0.0
az_sp = -9.6

DT = 0.01
alpha = 0.1  # smoothing factor

# === Helper Functions ===
def euler_to_quaternion(roll, pitch, yaw):
    cy, sy = np.cos(yaw*0.5), np.sin(yaw*0.5)
    cp, sp = np.cos(pitch*0.5), np.sin(pitch*0.5)
    cr, sr = np.cos(roll*0.5), np.sin(roll*0.5)
    return [
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ]

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

# === IMU Reader ===
def get_accels():
    msg = master.recv_match(type="RAW_IMU", blocking=True)
    if msg:
        return (
            msg.xacc * 9.80665 / 1000,
            msg.yacc * 9.80665 / 1000,
            msg.zacc * 9.80665 / 1000
        )
    return 0.0, 0.0, -9.8

# === Data Logging ===
time_log = []
ax_log, ay_log, az_log = [], [], []
pitch_log, roll_log, throttle_log = [], [], []

# === PID States ===
int_pitch = int_roll = int_throttle = 0.0
prev_err_pitch = prev_err_roll = prev_err_throttle = 0.0
ax_filt = ay_filt = az_filt = 0.0

# === Control Loop ===
start_time = time.time()
while time.time() - start_time < 60:
    ax_raw, ay_raw, az_raw = get_accels()

    # Low-pass filter
    ax = ax_filt = alpha * ax_raw + (1 - alpha) * ax_filt
    ay = ay_filt = alpha * ay_raw + (1 - alpha) * ay_filt
    az = az_filt = alpha * az_raw + (1 - alpha) * az_filt

    # === PITCH (az) PID ===
    err_pitch = az_sp - az
    int_pitch = np.clip(int_pitch + err_pitch * DT, -10, 10)
    d_pitch = (err_pitch - prev_err_pitch) / DT
    pitch_cmd = Kp_pitch * err_pitch + Ki_pitch * int_pitch + Kd_pitch * d_pitch
    pitch_cmd = np.clip(pitch_cmd, -math.radians(25), math.radians(25))
    prev_err_pitch = err_pitch

    # === ROLL (ay) PID ===
    err_roll = ay_sp - ay
    int_roll = np.clip(int_roll + err_roll * DT, -5, 5)
    d_roll = (err_roll - prev_err_roll) / DT
    roll_cmd = Kp_roll * err_roll + Ki_roll * int_roll + Kd_roll * d_roll
    roll_cmd = np.clip(roll_cmd, -math.radians(25), math.radians(25))
    prev_err_roll = err_roll

    # === THROTTLE (ax) PID ===
    err_throttle = ax_sp - ax
    int_throttle = np.clip(int_throttle + err_throttle * DT, -10, 10)
    d_throttle = (err_throttle - prev_err_throttle) / DT
    throttle_cmd = Kp_throttle * err_throttle + Ki_throttle * int_throttle + Kd_throttle * d_throttle
    throttle_cmd = np.clip(0.1 + throttle_cmd, 0.2, 0.7)
    prev_err_throttle = err_throttle

    # === SEND CMD ===
    send_attitude_cmd(roll_cmd, pitch_cmd, throttle_cmd)

    # === Log ===
    t_now = time.time() - start_time
    time_log.append(t_now)
    ax_log.append(ax)
    ay_log.append(ay)
    az_log.append(az)
    pitch_log.append(math.degrees(pitch_cmd))
    roll_log.append(math.degrees(roll_cmd))
    throttle_log.append(throttle_cmd)

    print(f"[t] {t_now:.2f}s | ax: {ax:.2f} | ay: {ay:.2f} | az: {az:.2f} | pitch: {math.degrees(pitch_cmd):.2f}° | roll: {math.degrees(roll_cmd):.2f}° | throttle: {throttle_cmd:.2f}")
    time.sleep(DT)

# === PLOTS ===
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(time_log, ax_log, label="ax")
plt.plot(time_log, [ax_sp]*len(time_log), 'r--', label="ax_sp")
plt.ylabel("ax (m/s²)")
plt.legend(); plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time_log, ay_log, label="ay")
plt.plot(time_log, [ay_sp]*len(time_log), 'r--', label="ay_sp")
plt.ylabel("ay (m/s²)")
plt.legend(); plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time_log, az_log, label="az")
plt.plot(time_log, [az_sp]*len(time_log), 'r--', label="az_sp")
plt.xlabel("Time (s)"); plt.ylabel("az (m/s²)")
plt.legend(); plt.grid()
plt.tight_layout(); plt.show()

# === COMMAND PLOTS ===
plt.figure(figsize=(12, 6))
plt.subplot(3, 1, 1)
plt.plot(time_log, pitch_log, label="Pitch (°)")
plt.grid(); plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_log, roll_log, label="Roll (°)")
plt.grid(); plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_log, throttle_log, label="Throttle")
plt.xlabel("Time (s)")
plt.grid(); plt.legend()

plt.tight_layout()
plt.show()
