import numpy as np
import time
from pymavlink import mavutil
import math
import matplotlib.pyplot as plt
import pandas as pd

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

# === Helpers ===
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

def get_accels():
    msg = master.recv_match(type="RAW_IMU", blocking=True)
    if msg:
        return (
            msg.xacc * 9.80665 / 1000,
            msg.yacc * 9.80665 / 1000,
            msg.zacc * 9.80665 / 1000
        )
    return 0.0, 0.0, -9.8

# === Logging ===
time_log = []
ax_log, ay_log, az_log = [], [], []
throttle_log = []

alpha = 0.1
ax_filt = 0.0
DT = 0.01

# === Step Parameters ===
step_time = 10.0  # seconds
throttle_low = 0.3
throttle_high = 0.6

print("🚀 Running step input test...")
start_time = time.time()
while time.time() - start_time < 60:
    t_now = time.time() - start_time

    # IMU
    ax_raw, ay_raw, az_raw = get_accels()
    ax = ax_filt = alpha * ax_raw + (1 - alpha) * ax_filt

    # Step input logic
    if t_now < step_time:
        throttle = throttle_low
    else:
        throttle = throttle_high

    # Send throttle-only command (neutral attitude)
    send_attitude_cmd(0.0, 0.0, throttle)

    # Log
    time_log.append(t_now)
    ax_log.append(ax)
    ay_log.append(ay_raw)
    az_log.append(az_raw)
    throttle_log.append(throttle)

    print(f"[{t_now:.2f}s] ax: {ax:.2f} | throttle: {throttle:.3f}")
    time.sleep(DT)

# === Save CSV ===
print("💾 Saving to 'stepinput_throttle_ax.csv'")
df = pd.DataFrame({
    'time': time_log,
    'throttle': throttle_log,
    'ax': ax_log,
    'ay': ay_log,
    'az': az_log
})
df.to_csv("stepinput_throttle_ax.csv", index=False)

# === Plot ===
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(time_log, throttle_log, label="Throttle")
plt.axvline(step_time, color='r', linestyle='--', label="Step Time")
plt.ylabel("Throttle"); plt.grid(); plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time_log, ax_log, label="ax (m/s²)")
plt.axvline(step_time, color='r', linestyle='--')
plt.ylabel("ax"); plt.xlabel("Time (s)")
plt.grid(); plt.legend()

plt.tight_layout()
plt.show()
