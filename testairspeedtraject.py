import time
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil

# Connect to the Vehicle
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Initial guidance parameters
x0 = 20.0   # initial horizontal separation (m)
z0 = -10.0  # initial vertical separation (m) (negative = platform below UAV)
vux0 = 25.0  # initial airspeed
vmin = 15.0  # minimum landing airspeed

# Compute constants once
k = (0.01 - x0) / -math.log(0.01 / -z0)
kv = x0 / -math.log(2 - vux0 / vmin)

print(f"Guidance constants: k = {k:.2f}, kv = {kv:.2f}")

# Simulate approach
dx = x0
for t in range(60):  # 60 steps (6 seconds at 10 Hz)
    dx = max(0.01, x0 - t * 5.0)  # decrease 5m per step

    # Guidance logic
    yu = z0 * (1 - math.exp(-(dx - x0) / k)) + 100 # desired altitude
    vux = vmin * (2 - math.exp(-dx / kv))     # desired airspeed

    print(f"[t={t:02d}] dx={dx:.1f} → alt={yu:.2f} m, airspeed={vux:.2f} m/s")

    # Send airspeed command (DO_CHANGE_SPEED)
    vehicle.message_factory.command_long_send(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        1, vux, -1, 0, 0, 0, 0
    )

    # Get current location
    location = vehicle.location.global_relative_frame
    lat = location.lat
    lon = location.lon

    # Send dynamic waypoint at same lat/lon, new altitude
    msg = vehicle.message_factory.mission_item_encode(
        0, 0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 1,
        0, 0, 0, 0,
        lat, lon, yu
    )
    vehicle.send_mavlink(msg)

    time.sleep(0.1)

print("✅ DroneKit landing guidance test complete.")
vehicle.close()
