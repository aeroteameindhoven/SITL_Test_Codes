#!/usr/bin/env python3
"""
Test script to:
1. Set an airspeed
2. ARM the vehicle
3. Take off to 30 meters altitude
4. Perform a serpentine motion using lateral acceleration
5. Stop external control after the test
"""

import time
from pymavlink import mavutil

# === CONFIGURATION ===
TARGET_AIRSPEED = 15.0  # Desired airspeed in m/s
TARGET_ALTITUDE = 100  # Takeoff altitude in meters
WAVE_ACCEL = 5          # Peak lateral acceleration (m/s²)
WAVE_PERIOD = 10         # Seconds per wave cycle (adjust for frequency)
NUM_WAVES = 10          # Total number of left/right waves
SYSTEM_ID = 1           # Target system ID
COMPONENT_ID = 1        # Target component ID

# === CONNECT TO VEHICLE ===
print("Connecting to vehicle...")
connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")  # Change if using serial
connection.wait_heartbeat()
print("✅ Heartbeat received! Connected to ArduPilot.")

def arm_vehicle():
    """ Send ARM command to ArduPilot """
    print("🔄 Arming vehicle...")
    connection.arducopter_arm()
    connection.motors_armed_wait()
    print("✅ Vehicle ARMED!")

def set_mode(mode):
    """ Set flight mode """
    print(f"🔄 Changing mode to {mode}...")
    connection.set_mode(mode)
    time.sleep(2)

def takeoff(altitude):
    """ Initiate AUTO takeoff """
    print(f"🚀 Taking off to {altitude}m...")
    connection.mav.command_long_send(
        SYSTEM_ID, COMPONENT_ID,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Confirmation
        0,  # Empty param
        0,  # Minimum pitch (degrees)
        0,  # Empty param
        0,  # Yaw angle (degrees)
        0,  # Latitude (ignored)
        0,  # Longitude (ignored)
        altitude  # Target altitude
    )
    time.sleep(10)  # Wait for takeoff
    print(f"✅ Takeoff complete! Reached {altitude}m")

def set_airspeed(airspeed):
    """ Set target airspeed """
    print(f"⚡ Setting airspeed to {airspeed} m/s...")
    connection.mav.command_long_send(
        SYSTEM_ID, COMPONENT_ID,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,  # Confirmation
        0,  # Speed type (0 = airspeed, 1 = ground speed)
        airspeed,  # Target speed (m/s)
        -1,  # No throttle change
        0,  # Empty param
        0,  # Empty param
        0,  # Empty param
        0   # Empty param
    )
    time.sleep(3)
    print(f"✅ Airspeed set to {airspeed} m/s")

def send_lateral_acceleration(accel, enable_external):
    """
    Send a LATERAL_ACCELERATION_COMMAND message.
    :param accel: Lateral acceleration (m/s²)
    :param enable_external: External control (1=enable, 0=disable)
    """
    connection.mav.lateral_acceleration_command_send(
        accel, enable_external, SYSTEM_ID, COMPONENT_ID
    )
    print(f"➡️ Sent lateral accel: {accel} m/s² | External Control: {'ON' if enable_external else 'OFF'}")

def set_dummy_waypoint():
    """ Set a far waypoint to keep L1 controller active. """
    print("📍 Setting dummy waypoint...")
    lat, lon, alt = 35.000000, -120.000000, TARGET_ALTITUDE  # Adjust as needed

    connection.mav.mission_item_send(
        SYSTEM_ID, COMPONENT_ID,
        0,  # Mission sequence number
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,  # Current = 2 (sets this as active)
        0,  # Autocontinue
        0, 0, 0, 0,  # Empty params
        lat, lon, alt
    )

    print(f"✅ Dummy waypoint set at {lat}, {lon}, {alt}m")
    time.sleep(2)

# === MISSION START ===
set_mode("GUIDED")
# set_airspeed(TARGET_AIRSPEED)  # Set airspeed before takeoff
arm_vehicle()
takeoff(TARGET_ALTITUDE)

# === SERPENTINE TEST: ALTERNATE LEFT/RIGHT MOTION ===
print("\n⚙️ Performing serpentine motion...")

set_dummy_waypoint()

# Enable external control before the motion starts
send_lateral_acceleration(0, 1)
time.sleep(2)

# Loop to create alternating acceleration (left/right turns)
for i in range(NUM_WAVES):
    send_lateral_acceleration(WAVE_ACCEL, 1)  # Right turn
    time.sleep(WAVE_PERIOD / 2)  # Wait for half wave period
    send_lateral_acceleration(-WAVE_ACCEL, 1)  # Left turn
    time.sleep(WAVE_PERIOD / 2)  # Wait for half wave period

# Stop external control at the end
send_lateral_acceleration(0.0, 0)
print("\n✅ Serpentine test completed successfully! 🎯")
