import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from pymavlink import mavutil
from geopy.distance import geodesic
import math
import threading
import matplotlib.pyplot as plt
import csv
import os

# --- PID Controller ---
class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
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

# ... [KEEP PID, quaternion helpers, imports, etc. same as before] ...
class GpsFollower(Node):
    def __init__(self):
        super().__init__('open_loop_uav')

        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        self.fixed_altitude = 20.0
        self.start_time = time.time()
        self.control_start_time = None

        self.log_time = []
        self.log_pitch_cmd = []
        self.log_throttle_cmd = []
        self.log_lateral_offset = []
        self.log_altitude = []
        self.log_airspeed = []

        self.takeoff(self.fixed_altitude)

        curr = self.vehicle.location.global_relative_frame
        target = self.offset_gps(curr.lat, curr.lon, 0, 100)
        self.vehicle.simple_goto(LocationGlobalRelative(-35.35040034, 149.16524926, self.fixed_altitude))
        self.get_logger().info("Going to distant waypoint (100m north)...")

        self.start_attitude_thread()

    def offset_gps(self, lat, lon, bearing_deg, distance_m):
        R = 6378137.0
        bearing = math.radians(bearing_deg)
        new_lat = math.asin(
            math.sin(math.radians(lat)) * math.cos(distance_m / R) +
            math.cos(math.radians(lat)) * math.sin(distance_m / R) * math.cos(bearing)
        )
        new_lon = math.radians(lon) + math.atan2(
            math.sin(bearing) * math.sin(distance_m / R) * math.cos(math.radians(lat)),
            math.cos(distance_m / R) - math.sin(math.radians(lat)) * math.sin(new_lat)
        )
        return math.degrees(new_lat), math.degrees(new_lon)

    def lateral_offset_from_heading(self, ref_lat, ref_lon, heading_rad, uav_lat, uav_lon):
        d_north = geodesic((ref_lat, ref_lon), (uav_lat, ref_lon)).meters
        d_east = geodesic((ref_lat, ref_lon), (ref_lat, uav_lon)).meters
        if uav_lat < ref_lat:
            d_north *= -1
        if uav_lon < ref_lon:
            d_east *= -1

        # Leftward unit vector
        leftward_x = -math.cos(heading_rad)
        leftward_y = math.sin(heading_rad)

        return d_east * leftward_x + d_north * leftward_y
    
    def apriltag_callback(self, msg):
        self.latest_apriltag_pose = msg.pose
        self.last_apriltag_time = time.time()

def attitude_loop(self):
    while rclpy.ok():
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if self.vehicle.mode.name != "GUIDED":
            time.sleep(0.05)
            continue

        current = self.vehicle.location.global_relative_frame
        uav_lat = current.lat
        uav_lon = current.lon
        uav_alt = current.alt or self.fixed_altitude
        airspeed = self.vehicle.airspeed or 0.0

        use_cv = self.latest_apriltag_pose is not None and (now - self.last_apriltag_time) < 0.5

        if use_cv:
            pose = self.latest_apriltag_pose
            lateral_error = pose.position.x  # left/right offset
            distance_error = pose.position.z - self.target_distance
        else:
            if self.target_lat is None or self.target_lon is None:
                time.sleep(0.05)
                continue

            distance = geodesic((uav_lat, uav_lon), (self.target_lat, self.target_lon)).meters
            distance_error = distance - self.target_distance

            d_north = geodesic((self.target_lat, self.target_lon), (uav_lat, self.target_lon)).meters
            d_east = geodesic((self.target_lat, self.target_lon), (self.target_lat, uav_lon)).meters
            if uav_lat < self.target_lat:
                d_north *= -1
            if uav_lon < self.target_lon:
                d_east *= -1
            lateral_error = d_east  # assuming car is going north

        target_airspeed = 16.0 + self.distance_pid.update(distance_error, dt)
        target_airspeed = max(10.0, min(22.0, target_airspeed))

        airspeed_error = target_airspeed - airspeed
        throttle_adjust = self.airspeed_pid.update(airspeed_error, dt)
        throttle_cmd = self.base_throttle + throttle_adjust
        throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

        altitude_error = self.fixed_altitude - uav_alt
        pitch_cmd = self.altitude_pid.update(altitude_error, dt)
        pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))

        roll_cmd = self.lateral_pid.update(lateral_error, dt)
        roll_cmd = max(min(roll_cmd, math.radians(3.0)), math.radians(-3.0))

        q = euler_to_quaternion(roll_cmd, pitch_cmd, 0.0)
        self.vehicle._master.mav.set_attitude_target_send(
            int((now - self.last_time) * 1000),
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            0b00000100,
            q,
            0.0, 0.0, 0.0,
            throttle_cmd
        )

        time.sleep(0.01)

    def takeoff(self, target_altitude):
        self.get_logger().info("Arming UAV...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            self.get_logger().info("Waiting for UAV to arm...")
            time.sleep(1)

        self.get_logger().info(f"Taking off to {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            self.get_logger().info(f"Altitude: {alt:.1f}m")
            if alt >= target_altitude * 0.95:
                self.get_logger().info("Reached target altitude!")
                break
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()

    try:
        rclpy.spin(gps_follower)
    except KeyboardInterrupt:
        pass
    finally:
        gps_follower.destroy_node()

        with open("open_loop_attitude_log.csv", "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "Time (s)", "Pitch Command (deg)", "Throttle Command",
                "Lateral Offset (m)", "Altitude (m)", "Airspeed (m/s)"
            ])
            for i in range(len(gps_follower.log_time)):
                writer.writerow([
                    gps_follower.log_time[i],
                    gps_follower.log_pitch_cmd[i],
                    gps_follower.log_throttle_cmd[i],
                    gps_follower.log_lateral_offset[i],
                    gps_follower.log_altitude[i],
                    gps_follower.log_airspeed[i]
                ])
        print("✅ CSV saved: open_loop_attitude_log.csv")

if __name__ == '__main__':
    main()
