from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Connect to the vehicle
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Function to change airspeed
def change_speed(airspeed):
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 0, airspeed, -1, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to set pitch attitude directly
def set_pitch(pitch_angle):
    """Set pitch angle (radians) for the aircraft."""
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,  # Time_boot_ms, Target system, Target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame
        0b00000100,  # Mask (only pitch active)
        [0, math.sin(pitch_angle / 2), 0, math.cos(pitch_angle / 2)],  # Quaternion (only pitch)
        0, 0, 0, 0  # Body rates + thrust (ignored)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to reduce throttle
def reduce_throttle(throttle_percent):
    """Reduce throttle to mimic landing flare behavior."""
    pwm_value = int(1100 + (throttle_percent / 100.0) * 800)  # Scale from 1100 (idle) to 1900 (max)
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, 5, pwm_value, 0, 0, 0, 0, 0  # Servo channel 5 (change if needed)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Set GUIDED mode and arm
print("Setting mode to GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

print("Arming vehicle...")
vehicle.armed = True
time.sleep(2)

# Takeoff sequence
target_altitude = 100  # meters
print(f"Taking off to {target_altitude}m altitude...")
vehicle.simple_takeoff(target_altitude)

# Wait for the UAV to reach target altitude
while True:
    alt = vehicle.location.global_relative_frame.alt
    if alt >= target_altitude * 0.95:
        break
    time.sleep(0.1)

# Setup for straight-line trajectory
car_speed = 16.67  # m/s (60 km/h)
car_heading = math.radians(90)  # Moving east
step_distance = 0.2  # Move 20 cm per update

# Get starting reference position
ref_lat = vehicle.location.global_relative_frame.lat
ref_lon = vehicle.location.global_relative_frame.lon
deg_per_meter_lat = 1 / 111320
deg_per_meter_lon = 1 / (111320 * math.cos(math.radians(ref_lat)))

# Set initial speed
change_speed(car_speed)

print("Following the car at constant speed...")
current_lat, current_lon = ref_lat, ref_lon

while True:
    # Get current altitude
    alt = vehicle.location.global_relative_frame.alt

    # Compute next position regardless of flare
    current_lat += step_distance * math.cos(car_heading) * deg_per_meter_lat
    current_lon += step_distance * math.sin(car_heading) * deg_per_meter_lon
    target_location = LocationGlobalRelative(current_lat, current_lon, target_altitude)

    if alt > target_altitude:
        print(f"[FLARE MODE] Altitude {alt:.1f}m > {target_altitude}m. Slowing down...")
        
        # Reduce throttle to 20% (simulate flare)
        reduce_throttle(20)

        # Slightly pitch up (flare effect)
        set_pitch(math.radians(5))

        # Reduce airspeed gradually
        change_speed(car_speed * 0.6)  # Reduce to 60% of car speed

    # **Always send updated waypoint, even in flare mode**
    vehicle.simple_goto(target_location)

    # 100Hz update rate
    time.sleep(0.01)

