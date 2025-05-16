from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math
from geopy.distance import geodesic

# --- PID Controller ---
class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# --- Quaternion Helper ---
def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ]

# --- Speed Scheduling Function ---
def speed_from_distance(dist, max_speed=20.0, min_speed=5.0, slowdown_radius=300):
    if dist > slowdown_radius:
        return max_speed
    else:
        return min_speed + (max_speed - min_speed) * (dist / slowdown_radius)

# --- Connect to vehicle ---
print("Connecting to vehicle...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Connected!")

# --- Set your guided mode target here (clicked point from MAVProxy map) ---
guided_target = (-35.34335724 , 149.14447906)  # Replace with your lat/lon

# --- Target Altitude ---
target_altitude = 48.0  # m

# --- PID Controllers ---
airspeed_pid = PID(kp=0.012, ki=0.0083, kd=0.16)
altitude_pid = PID(kp=0.05, ki=0.01, kd=0.25, integral_limit=5.5)

# --- Timing ---
start_time = time.time()
last_time = start_time

# --- Estimate Initial Throttle ---
initial_throttle = 0.6
if hasattr(vehicle, 'channels') and '3' in vehicle.channels:
    try:
        pwm = int(vehicle.channels['3'])
        initial_throttle = (pwm - 1000) / 1000
    except:
        pass

# --- Prime throttle with attitude hold ---
q_init = euler_to_quaternion(0.0, 0.0, 0.0)
for _ in range(20):  # ~200ms warmup
    vehicle._master.mav.set_attitude_target_send(
        0,
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,
        q_init,
        0.0, 0.0, 0.0,
        initial_throttle
    )
    time.sleep(0.01)

# --- Initialize PID integral to match initial throttle ---
current_airspeed = vehicle.airspeed or 0.0
airspeed_error = 20.0 - current_airspeed
if airspeed_pid.ki > 0:
    airspeed_pid.integral = (initial_throttle - airspeed_pid.kp * airspeed_error) / airspeed_pid.ki

print("Running closed-loop control to waypoint...")
distance_to_wp = float("inf")

# --- Main Loop ---
while distance_to_wp > 5.0:  # stop when within 5 meters
    now = time.time()
    dt = now - last_time
    elapsed = now - start_time
    last_time = now

    current_airspeed = vehicle.airspeed or 0.0
    current_altitude = vehicle.location.global_relative_frame.alt or 0.0

    # --- Dynamic airspeed setpoint based on distance to guided target ---
    current_loc = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    distance_to_wp = geodesic(current_loc, guided_target).meters
    target_airspeed = speed_from_distance(distance_to_wp)

    # --- PID Control ---
    airspeed_error = target_airspeed - current_airspeed
    altitude_error = target_altitude - current_altitude

    throttle_cmd = airspeed_pid.update(airspeed_error, dt)
    throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

    pitch_cmd = altitude_pid.update(altitude_error, dt)
    pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))

    # --- Attitude Command ---
    q = euler_to_quaternion(0.0, pitch_cmd, 0.0)
    vehicle._master.mav.set_attitude_target_send(
        int(elapsed * 1000),
        vehicle._master.target_system,
        vehicle._master.target_component,
        0b00000100,
        q,
        0.0, 0.0, 0.0,
        throttle_cmd
    )

    print(f"[{int(elapsed * 1000)} ms] Dist2WP: {distance_to_wp:.1f} m | TargetAS: {target_airspeed:.2f} | AS: {current_airspeed:.2f} | Alt: {current_altitude:.2f} | Throttle: {throttle_cmd:.3f}")
    time.sleep(0.01)

print("Reached waypoint! 🎯 Holding position or taking further action.")
