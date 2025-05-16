from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import matplotlib.pyplot as plt

# Connect to the simulated vehicle (e.g., SITL)
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Function to update the vehicle's speed
def change_speed(airspeed):
    """Set the airspeed for continuous movement using a MAVLink command."""
    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                   # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # command
        0,                                      # confirmation
        0,                                      # speed type: 0 = airspeed, 1 = ground speed
        airspeed,                               # new speed (m/s)
        -1,                                     # throttle (-1 to leave unchanged)
        0, 0, 0, 0                              # unused parameters
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Set mode to GUIDED and arm the vehicle
print("Setting mode to GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

print("Arming vehicle...")
vehicle.armed = True
time.sleep(2)

# Take off to target altitude
target_altitude = 100  # meters
print(f"Taking off to {target_altitude}m altitude...")
vehicle.simple_takeoff(target_altitude)

# Wait until the UAV reaches target altitude
while True:
    alt = vehicle.location.global_relative_frame.alt
    if alt >= target_altitude * 0.95:
        break
    time.sleep(0.1)

# --- Straight-line Trajectory with Lateral Deviations ---
car_speed = 16.67  # m/s (60 km/h)
car_heading = math.radians(90)  # moving east (90°)

# Use current vehicle position as the starting reference
ref_lat = vehicle.location.global_relative_frame.lat
ref_lon = vehicle.location.global_relative_frame.lon

# Conversion factors (approximate)
deg_per_meter_lat = 1 / 111320
deg_per_meter_lon = 1 / (111320 * math.cos(math.radians(ref_lat)))

# Set vehicle speed to match the target speed
change_speed(17)

print("Following a straight-line trajectory with lateral deviations...")

# Lateral deviation parameters
amplitude = 0.1  # maximum lateral offset in meters
period = 50   # period of the oscillation (in meters along the path)

distance_traveled = 0.0

# --- Set Up Real-Time Plotting ---
plt.ion()  # interactive mode on
fig, ax = plt.subplots()
target_line, = ax.plot([], [], 'r-', label='Target Path')
actual_line, = ax.plot([], [], 'b-', label='Actual Path')
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.set_title("UAV Path: Target vs Actual")
ax.legend()

# Lists to store positions for plotting
target_lats = []
target_lons = []
actual_lats = []
actual_lons = []

# Only start plotting after the UAV is on the straight path
start_plotting = False

last_plot_time = time.time()

while True:
    # Move forward by a small step
    step_distance = 0.2  # meters per update
    distance_traveled += step_distance

    # Base position along the straight-line path
    base_lat = ref_lat + distance_traveled * math.cos(car_heading) * deg_per_meter_lat
    base_lon = ref_lon + distance_traveled * math.sin(car_heading) * deg_per_meter_lon

    # Compute lateral offset as a sine function of distance traveled
    lateral_offset = amplitude * math.sin(2 * math.pi * distance_traveled / period)
    
    # Perpendicular direction to car_heading: add 90° (pi/2 radians)
    perp_heading = car_heading + math.pi/2
    offset_lat = lateral_offset * math.cos(perp_heading) * deg_per_meter_lat
    offset_lon = lateral_offset * math.sin(perp_heading) * deg_per_meter_lon

    # Final target position with lateral deviation added
    target_lat = base_lat + offset_lat
    target_lon = base_lon + offset_lon

    # Command UAV to fly to the computed position at the target altitude
    target_location = LocationGlobalRelative(target_lat, target_lon, target_altitude)
    vehicle.simple_goto(target_location)

    # Get current (actual) UAV position
    actual_lat = vehicle.location.global_relative_frame.lat
    actual_lon = vehicle.location.global_relative_frame.lon

    # Start plotting only when the UAV is near the straight-line path
    if not start_plotting:
        distance_from_line = math.sqrt((actual_lat - base_lat)**2 + (actual_lon - base_lon)**2)
        if distance_from_line < 5 * deg_per_meter_lat:  # Close enough to the path
            start_plotting = True
            print("UAV has stabilized on the path. Starting visualization...")

    if start_plotting:
        # Record positions for plotting
        target_lats.append(target_lat)
        target_lons.append(target_lon)
        actual_lats.append(actual_lat)
        actual_lons.append(actual_lon)

        # Update plot every second
        if time.time() - last_plot_time >= 1.0:
            target_line.set_data(target_lons, target_lats)
            actual_line.set_data(actual_lons, actual_lats)

            # Keep the x-axis and y-axis range limited to focus on small deviations
            ax.set_xlim(min(target_lons) - 0.00005, max(target_lons) + 0.00005)
            ax.set_ylim(min(target_lats) - 0.00005, max(target_lats) + 0.00005)

            plt.draw()
            plt.pause(0.001)
            last_plot_time = time.time()

    # Maintain an update rate of about 100 Hz
    time.sleep(0.01)
