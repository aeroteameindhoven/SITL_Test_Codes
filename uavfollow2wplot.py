import math
import matplotlib.pyplot as plt
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Connect to the vehicle
print("Connecting to vehicle...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Set mode to GUIDED and arm the vehicle
print("Setting mode to GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

print("Arming vehicle...")
vehicle.armed = True
while not vehicle.armed:
    time.sleep(1)

# Constants
CAR_SPEED = 16.67  # Car's speed in m/s (equivalent to 60 km/h)
FOLLOW_DISTANCE = 20  # Distance ahead where the UAV should aim to follow
FREQ = 10  # Hz (waypoint update frequency, reduced to avoid spam)
DT = 1 / FREQ  # Time step per loop
EARTH_RADIUS = 6378137.0  # Earth radius in meters
ALTITUDE = 50  # Fixed altitude (meters)

# Take off to target altitude
print(f"Taking off to {ALTITUDE}m altitude...")
vehicle.simple_takeoff(ALTITUDE)

# Wait until the UAV reaches target altitude
while True:
    alt = vehicle.location.global_relative_frame.alt
    if alt >= ALTITUDE * 0.95:
        break
    time.sleep(0.1)

# Lists to store target and actual positions
target_latitudes = []
target_longitudes = []
target_altitudes = []

actual_latitudes = []
actual_longitudes = []
actual_altitudes = []

# Function to compute new target location
def get_location_ahead(current_location, heading, distance):
    lat = current_location.lat
    lon = current_location.lon
    delta_lat = (distance / EARTH_RADIUS) * (180 / math.pi)
    delta_lon = (distance / EARTH_RADIUS) * (180 / math.pi) / math.cos(math.radians(lat))
    new_lat = lat + delta_lat * math.cos(math.radians(heading))
    new_lon = lon + delta_lon * math.sin(math.radians(heading))
    return LocationGlobalRelative(new_lat, new_lon, ALTITUDE)

plt.ion()  # Enable interactive mode
fig1, ax1 = plt.subplots(figsize=(10, 5))
fig2, ax2 = plt.subplots(figsize=(10, 5))

print("Starting UAV pursuit of moving waypoint...")

while True:
    # Get current location and heading
    current_location = vehicle.location.global_relative_frame
    heading = vehicle.heading  # Heading in degrees

    # Calculate the moving waypoint ahead of the UAV
    moving_waypoint = get_location_ahead(current_location, heading, FOLLOW_DISTANCE)

    # Append target values
    target_latitudes.append(moving_waypoint.lat)
    target_longitudes.append(moving_waypoint.lon)
    target_altitudes.append(moving_waypoint.alt)

    # Get actual UAV position
    actual_latitudes.append(current_location.lat)
    actual_longitudes.append(current_location.lon)
    actual_altitudes.append(current_location.alt)

    # Send UAV to the moving waypoint
    vehicle.simple_goto(moving_waypoint)

    # Live updating plots
    ax1.clear()
    ax1.plot(target_longitudes, target_latitudes, label="Target Position", linestyle="--", marker="o")
    ax1.plot(actual_longitudes, actual_latitudes, label="Actual Position", linestyle="-", marker="x")
    ax1.set_xlabel("Longitude")
    ax1.set_ylabel("Latitude")
    ax1.set_title("Target vs Actual Position (Longitude vs Latitude)")
    ax1.legend()
    ax1.grid()
    
    ax2.clear()
    time_axis = [i * DT for i in range(len(target_altitudes))]
    ax2.plot(time_axis, target_altitudes, label="Target Altitude", linestyle="--", marker="o")
    ax2.plot(time_axis, actual_altitudes, label="Actual Altitude", linestyle="-", marker="x")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Altitude (m)")
    ax2.set_title("Target vs Actual Altitude over Time")
    ax2.legend()
    ax2.grid()
    
    plt.pause(DT)
