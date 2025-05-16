from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Connect to the simulated vehicle (e.g., SITL)

# UART_PORT = "/dev/ttyUSB0"  # Change this if needed
# BAUD_RATE = 57600

# This is to connect through simulation
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# print("Connecting to vehicle...")
# vehicle = connect(UART_PORT, baud=BAUD_RATE, wait_ready=True)

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
        0, 0, 0, 0                             # unused parameters
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

# --- Straight-line Trajectory ---
target_speed = 20  # m/s
target_heading = math.radians(90)  # moving east (90°)
step_distance = 0.28  # 20 cm per update

# Use current vehicle position as the starting reference
ref_lat = vehicle.location.global_relative_frame.lat
ref_lon = vehicle.location.global_relative_frame.lon
deg_per_meter_lat = 1 / 111320
deg_per_meter_lon = 1 / (111320 * math.cos(math.radians(ref_lat)))


print("Following a straight-line trajectory at constant speed...")
current_lat, current_lon = ref_lat, ref_lon
start_time = time.time()

while True:
    # Compute next position (move 20 cm per step)
    current_lat += step_distance * math.cos(target_heading) * deg_per_meter_lat
    current_lon += step_distance * math.sin(target_heading) * deg_per_meter_lon

    # Command UAV to fly to the next computed position
    target_location = LocationGlobalRelative(current_lat, current_lon, target_altitude)
    vehicle.simple_goto(target_location)

    # Maintain a 100Hz update ratee
    time.sleep(0.01)
