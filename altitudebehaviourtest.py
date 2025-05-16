from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import matplotlib.pyplot as plt

# Connect to the simulated vehicle (e.g., SITL)

UART_PORT = "/dev/ttyUSB0"  # Change this if needed
BAUD_RATE = 57600

# This is to connect through simulation
# print("Connecting to vehicle...")
# vehicle = connect('127.0.0.1:14550', wait_ready=True)

print("Connecting to vehicle...")
vehicle = connect(UART_PORT, baud=BAUD_RATE, wait_ready=True)

# Set mode to GUIDED and arm the vehicle
print("Setting mode to GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

print("Arming vehicle...")
vehicle.armed = True
time.sleep(2)

# Take off to a base altitude
base_altitude = 50.0  # Base altitude in meters
print(f"Taking off to {base_altitude}m altitude...")
vehicle.simple_takeoff(base_altitude)

# Wait until the UAV reaches the base altitude
while True:
    alt = vehicle.location.global_relative_frame.alt
    if alt >= base_altitude * 0.95:
        break
    time.sleep(0.5)

# Target waypoint (latitude and longitude)
waypoint_lat = 37.7749
waypoint_lon = -122.4194

# Altitude oscillation parameters
amplitude = 1  # Maximum altitude variation in meters
period = 100    # Oscillation period in seconds

# Setup matplotlib for real-time plotting
plt.ion()  # Interactive mode on
fig, ax = plt.subplots()
target_line, = ax.plot([], [], 'r-', label='Target Altitude')
actual_line, = ax.plot([], [], 'b-', label='Actual Altitude')
ax.set_xlabel("Time (s)")
ax.set_ylabel("Altitude (m)")
ax.set_title("Target vs Actual Altitude")
ax.legend()

# Lists to hold data for plotting
times = []
target_alts = []
actual_alts = []

start_time = time.time()
print("Starting altitude oscillation with graphing...")
while True:
    # Calculate elapsed time
    t = time.time() - start_time
    
    # Compute target altitude using a sine function
    target_alt = base_altitude + amplitude * math.sin(2 * math.pi * t / period)
    
    # Create target location (with fixed lat/lon but updated altitude)
    target_location = LocationGlobalRelative(waypoint_lat, waypoint_lon, target_alt)
    vehicle.simple_goto(target_location)
    
    # Read the actual altitude from the vehicle
    actual_alt = vehicle.location.global_relative_frame.alt
    
    # Log the data
    times.append(t)
    target_alts.append(target_alt)
    actual_alts.append(actual_alt)
    
    # Update the plot with new data
    target_line.set_data(times, target_alts)
    actual_line.set_data(times, actual_alts)
    ax.relim()
    ax.autoscale_view(True, True, True)
    plt.draw()
    plt.pause(0.1)
    
    # Wait a bit before next update
    time.sleep(1)
