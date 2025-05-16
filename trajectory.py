import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import threading
from pymavlink import mavutil

# === Load Trajectory Data from CSV ===
csv_filename = "trajectory_data.csv"
trajectory_data = pd.read_csv(csv_filename, header=None).values

t_values = trajectory_data[:, 0]      # Time column
lat_acc_values = trajectory_data[:, 7]  # Y acceleration → Lateral acceleration
climb_rate_values = trajectory_data[:, 6]  # Z velocity → Climb rate
vx_values = trajectory_data[:, 4]     # X velocity → Forward velocity (vx)

# === MAVLink Connection ===
print("Connecting to vehicle...")
connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
connection.wait_heartbeat()
print("✅ Heartbeat received! Connected to ArduPilot.")

target_system = connection.target_system
target_component = connection.target_component

# === Altitude Logging Storage ===
altitude_log = []

# === Function to Send MAVLink Commands ===
def send_lateral_acceleration(accel, enable_external):
    connection.mav.lateral_acceleration_command_send(
        accel, enable_external, target_system, target_component
    )
    print(f"➡️ Sent lateral accel: {accel:.2f} m/s² | External Control: {'ON' if enable_external else 'OFF'}")

def send_climb_rate(climb_rate, enable_external=1):
    msg = connection.mav.climb_rate_command_encode(
        float(climb_rate), int(enable_external), int(target_system), int(target_component)
    )
    connection.mav.send(msg)
    print(f"⤴️ Sent climb rate: {climb_rate:.2f} m/s")

def send_change_speed(vx):
    """Send DO_CHANGE_SPEED to set ground speed (vx)."""
    connection.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,       # Confirmation
        0,       # Speed type: 1 = groundspeed
        vx,      # Desired speed (m/s)
        -1, 0, 0, 0, 0  # Unused parameters
    )
    print(f"🚀 Sent DO_CHANGE_SPEED: {vx:.2f} m/s")

def get_current_altitude():
    """Returns current relative altitude in meters from GLOBAL_POSITION_INT."""
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        return msg.relative_alt / 1000.0  # mm to meters
    return None

# === Function to Execute Trajectory ===
def execute_trajectory():
    print("[+] Running trajectory simulation...")
    start_time = time.monotonic()

    for i in range(len(t_values) - 1):
        lat_acc = float(lat_acc_values[i])
        climb_rate = float(climb_rate_values[i])
        vx = float(vx_values[i])

        # Send MAVLink commands
        #send_lateral_acceleration(lat_acc, enable_external=1)
        #send_climb_rate(climb_rate, enable_external=1)
        send_change_speed(vx + 19)

        # Log altitude
        current_altitude = get_current_altitude()
        if current_altitude is not None:
            elapsed = time.monotonic() - start_time
            altitude_log.append((elapsed, current_altitude))

        # Sleep until next time step
        if i < len(t_values) - 1:
            next_time = start_time + t_values[i + 1] - t_values[0]
            while time.monotonic() < next_time:
                time.sleep(0.001)  # Small sleep for precise timing

    print("[+] Trajectory simulation complete.")

# === Start Execution Thread ===
trajectory_thread = threading.Thread(target=execute_trajectory)
trajectory_thread.start()
trajectory_thread.join()

# === Plot Altitude Over Time ===
if altitude_log:
    times, altitudes = zip(*altitude_log)
    plt.figure(figsize=(10, 5))
    plt.plot(times, altitudes, label='UAV Altitude (m)')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('UAV Altitude During Trajectory')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
else:
    print("⚠️ No altitude data was recorded.")
