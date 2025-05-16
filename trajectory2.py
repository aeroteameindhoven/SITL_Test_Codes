import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import threading
from pymavlink import mavutil

# === Load Trajectory Data from CSV ===
csv_filename = "trajectory_data.csv"
trajectory_data = pd.read_csv(csv_filename, header=None).values

t_values = trajectory_data[:, 0]      # Time
vx_rel_values = trajectory_data[:, 7] # X velocity delta (forward)
vy_rel_values = trajectory_data[:, 5]# Y acceleration delta (lateral)
vz_rel_values = trajectory_data[:, 6] # Z acceleration delta (vertical)

# === MAVLink Connection ===
print("Connecting to vehicle...")
connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
connection.wait_heartbeat()
print("✅ Heartbeat received! Connected to ArduPilot.")

target_system = connection.target_system
target_component = connection.target_component

# === Shared state ===
current_airspeed = 13.0  # default fallback
airspeed_lock = threading.Lock()

# === Thread: Listen for Airspeed from VFR_HUD ===
def listen_airspeed():
    global current_airspeed
    while not trajectory_thread_done:
        msg = connection.recv_match(type="VFR_HUD", blocking=True, timeout=1.0)
        if msg:
            with airspeed_lock:
                current_airspeed = 14

# === Velocity to Attitude + Throttle Controller ===
def acceleration_to_attitude(a_y, a_z, g=9.81):
    roll = np.arctan2(a_y, g)
    alpha = np.radians(5.0)  # Trim AoA
    pitch = np.arcsin(a_z / g)
    return roll, pitch


# === Euler to Quaternion Conversion ===
def euler_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]

# === Send Attitude Setpoint ===
def send_attitude_target(roll, pitch, thrust):
    yaw = 0.0
    q = euler_to_quaternion(roll, pitch, yaw)
    time_boot_ms = int((time.monotonic() - log_start_time) * 1000)

    connection.mav.set_attitude_target_send(
        time_boot_ms,
        target_system,
        target_component,
        0b10000111,  # Ignore body rates, use attitude and thrust
        q,
        0, 0, 0,
        thrust
    )
    print(f"🎯 Roll: {np.degrees(roll):.1f}°, Pitch: {np.degrees(pitch):.1f}°, Thrust: {thrust:.2f}")

# === Logging ===
actual_altitudes = []
x_positions = []
logged_times = []
trajectory_thread_done = False
log_start_time = time.monotonic()
lat0, lon0 = None, None

# === Log Altitude + X Displacement ===
def log_position():
    global lat0, lon0
    while not trajectory_thread_done:
        msg = connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
        if msg:
            alt_m = msg.relative_alt / 1000.0  # mm → m
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            timestamp = time.monotonic() - log_start_time

            if lat0 is None or lon0 is None:
                lat0, lon0 = lat, lon

            dx = (lon - lon0) * 111320 * np.cos(np.radians(lat0))

            actual_altitudes.append(alt_m)
            x_positions.append(dx)
            logged_times.append(timestamp)

def send_speed_command(airspeed_mps):
    connection.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1,  # Speed type: airspeed
        airspeed_mps,
        -1,  # throttle unchanged
        0, 0, 0, 0
    )
    print(f"🚀 Target Airspeed: {airspeed_mps:.2f} m/s")


# === Execute Trajectory ===
def execute_trajectory():
    print("[+] Running trajectory simulation...")
    start_time = time.monotonic()

    for i in range(len(t_values) - 1):
        vx_rel = float(vx_rel_values[i])
        a_y = float(vy_rel_values[i])
        a_z = float(vz_rel_values[i])

        # === Compute and send attitude ===
        roll, pitch = acceleration_to_attitude(a_y, a_z)
        send_attitude_target(roll, pitch, thrust=0)  # thrust ignored

        # === Compute and send airspeed ===
        with airspeed_lock:
            current = current_airspeed
        desired_airspeed = current + vx_rel
        send_speed_command(desired_airspeed)

        # === Timing ===
        if i < len(t_values) - 1:
            next_time = start_time + t_values[i + 1] - t_values[0]
            while time.monotonic() < next_time:
                time.sleep(0.001)

    print("[+] Trajectory simulation complete.")


# === Start Threads ===
trajectory_thread = threading.Thread(target=execute_trajectory)
position_thread = threading.Thread(target=log_position)
airspeed_thread = threading.Thread(target=listen_airspeed)

position_thread.start()
airspeed_thread.start()
trajectory_thread.start()

trajectory_thread.join()
trajectory_thread_done = True
position_thread.join()
airspeed_thread.join()

# === Plot Altitude ===
print("📊 Plotting actual altitude...")
plt.figure(figsize=(10, 4))
plt.plot(logged_times, actual_altitudes, label="Altitude (m)", color='blue')
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("UAV Altitude During Trajectory")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# === Plot Horizontal X Displacement ===
print("📊 Plotting horizontal X displacement...")
plt.figure(figsize=(10, 4))
plt.plot(logged_times, x_positions, label="X Displacement (m)", color='green')
plt.xlabel("Time (s)")
plt.ylabel("X Position (m)")
plt.title("UAV Forward Displacement Over Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
