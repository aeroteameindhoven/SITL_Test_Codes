from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Connect to the vehicle
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def change_speed(airspeed):
    """Set the airspeed for continuous movement"""
    print(f"Setting airspeed to {airspeed} m/s")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # Command
        0,  # Confirmation
        0,  # Speed type (0 = airspeed, 1 = ground speed)
        airspeed,  # New speed (m/s)
        -1,  # Throttle (-1 = keep current)
        0, 0, 0, 0  # Unused parameters
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def move_forward(distance_m=10):
    """Move forward relative to the UAV's current heading."""
    current_location = vehicle.location.global_relative_frame
    heading = math.radians(vehicle.heading)  # Convert heading to radians

    # Approximate conversion factors
    deg_per_meter_lat = 1 / 111320  # 1° latitude ~ 111.32 km
    deg_per_meter_lon = 1 / (111320 * math.cos(math.radians(current_location.lat)))  # Adjust for longitude

    # Compute new latitude/longitude based on heading
    new_lat = current_location.lat + (distance_m * math.cos(heading) * deg_per_meter_lat)
    new_lon = current_location.lon + (distance_m * math.sin(heading) * deg_per_meter_lon)

    print(f"Moving forward to Lat={new_lat}, Lon={new_lon}, Alt={current_location.alt}")

    msg = vehicle.message_factory.mission_item_encode(
        0, 0, 0,  # Target system, target component, sequence
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Coordinate frame
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Waypoint command
        2,  # Current waypoint (2 = execute immediately)
        0,  # Autocontinue
        0, 0, 0, 0,  # Unused parameters
        new_lat, new_lon, current_location.alt  # Target lat, lon, alt
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

# Set mode to GUIDED
print("Setting mode to GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

# Arm and take off
print("Arming...")
vehicle.armed = True
time.sleep(2)

print("Taking off to 100m altitude...")
vehicle.simple_takeoff(100)

# Wait until UAV reaches target altitude
while True:
    alt = vehicle.location.global_relative_frame.alt
    print(f"Altitude: {alt:.2f}m")
    if alt >= 95:  # Target is 100m, consider 95m as "good enough"
        print("Reached target altitude")
        break
    time.sleep(1)

# Set speed and start movement
change_speed(5)  # Increase airspeed to 15 m/s

print("Starting continuous forward movement...")
while True:
    move_forward(distance_m=10)  # Move forward by 5 meters per update
    time.sleep(0.1)  # Update faster (0.5 seconds instead of 2