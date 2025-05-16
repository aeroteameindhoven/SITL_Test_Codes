import numpy as np
import time
from pymavlink import mavutil
import matplotlib.pyplot as plt
import math
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

# === PID GAINS (Insert yours from MATLAB)
Kp = 0.0
Ki = 0.03998
Kd = 0.00

# === Control Params
ax_sp = 1.0        # Desired ax in m/s²
DT = 0.01          # 100 Hz
alpha = 0.1        # Low-pass filter for smoothing
throttle_bias = 0.4

# === State Vars
ax_filt = 0.0
int_ax = 0.0
prev_err_ax = 0.0

# === Logging
time_log, ax_log, throttle_log = [], [], []

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

def send_throttle_only(throttle):
    q = euler_to_quaternion(0.0, 0.0, 0.0)
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

def get_ax():
    msg = master.recv_match(type="RAW_IMU", blocking=True)
    if msg:
        return msg.xacc * 9.80665 / 1000
    return 0.0

# === Control Loop ===
print("🚀 Running ax PID loop (Throttle only)...")
start_time = time.time()
while time.time() - start_time < 60:
    t = time.time() - start_time
    ax_raw = get_ax()
    ax = ax_filt = alpha * ax_raw + (1 - alpha) * ax_filt

    # PID control
    err = ax_sp - ax
    int_ax = np.clip(int_ax + err * DT, -10, 10)
    d_ax = (err - prev_err_ax) / DT
    throttle_cmd = throttle_bias + (Kp * err + Ki * int_ax + Kd * d_ax)
    throttle_cmd = np.clip(throttle_cmd, 0.2, 0.7)
    prev_err_ax = err

    send_throttle_only(throttle_cmd)

    # Log
    time_log.append(t)
    ax_log.append(ax)
    throttle_log.append(throttle_cmd)

    print(f"[{t:.2f}s] ax: {ax:.2f} | err: {err:.2f} | throttle: {throttle_cmd:.3f}")
    time.sleep(DT)

# === Save Data ===
print("💾 Saving log to ax_pid_log.csv")
df = pd.DataFrame({
    'time': time_log,
    'ax': ax_log,
    'throttle': throttle_log
})
df.to_csv("ax_pid_log.csv", index=False)

# === Plot ===
plt.figure(figsize=(10, 5))
plt.subplot(2,1,1)
plt.plot(time_log, ax_log, label="ax (m/s²)")
plt.axhline(ax_sp, color='r', linestyle='--', label="Setpoint")
plt.ylabel("ax")
plt.grid(); plt.legend()

plt.subplot(2,1,2)
plt.plot(time_log, throttle_log, label="Throttle")
plt.ylabel("Throttle"); plt.xlabel("Time (s)")
plt.grid(); plt.legend()

plt.tight_layout()
plt.show()
