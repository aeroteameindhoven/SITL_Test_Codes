import time
import numpy as np
from pymavlink import mavutil

# Connect to the UAV
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print(f"Connected to system {master.target_system}")

# Descent profile
altitude_start = 10       # in meters above home
altitude_end = 0          # landing
duration = 1.25           # seconds to descend
dt = 0.1
t_vals = np.arange(0, duration + dt, dt)

# Send airspeed target (recommended for TECS + fixed-wing)
airspeed = 16  # m/s
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
    0,
    1,              # Airspeed type
    airspeed,       # Target airspeed
    -1, 0, 0, 0, 0
)
print(f"Target airspeed set to {airspeed} m/s")

# Send initial GUIDED mode enable
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# Start descent using altitude-only GLOBAL_INT
print("Starting descent...")
for t in t_vals:
    alt = altitude_start + (altitude_end - altitude_start) * (t / duration)
    alt_mm = int(alt * 1000)  # convert to millimeters
    time_boot_ms = int((time.time() * 1000) % 4294967295)

    master.mav.set_position_target_global_int_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b111111111000,  # ignore everything except alt
        0, 0, alt_mm,     # dummy lat/lon, alt in mm
        0, 0, 0,          # vx, vy, vz
        0, 0, 0,          # afx, afy, afz
        0, 0              # yaw, yaw_rate
    )

    print(f"[{t:.2f}s] Target REL altitude: {alt:.2f} m")
    time.sleep(dt)

print("Descent complete.")
