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
from apriltag_interfaces.msg import TagPoseStamped  # Your custom message

# --- PID Controller ---
class TunablePID:
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

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

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

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_dronekit')

        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 15)
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)
        self.create_subscription(TagPoseStamped, '/apriltag/pose_in_base', self.apriltag_callback, 10)

        self.last_apriltag_time = 0.0
        self.latest_apriltag_pose = None
        self.fixed_altitude = 10.0
        self.last_lat, self.last_lon = None, None
        self.uav_lat, self.uav_lon = None, None
        self.prev_tag_source = "GPS"
        self.last_time = time.time()
        self.start_time = self.last_time
        self.latest_tags = {}

        self.prius_last_lat = None
        self.prius_last_lon = None


        # PID controllers
        self.airspeed_pid = PID(kp=0.08, ki=0.04, kd=0.0, integral_limit=5.0)
        self.altitude_pid = PID(kp=0.052, ki=0.0095, kd=0.25, integral_limit=5.5)
        #self.altitude_pid = PID(kp=0.065, ki=0.01, kd=0.05, integral_limit=4.5)
        self.distance_pid = PID(kp=0.1, ki=0.0, kd=0.0, integral_limit=5.0)
        #self.lateral_pid = PID(kp=0.0011, ki=0.0006, kd=0.13, integral_limit=2.0)
        #self.lateral_pid = PID(kp=0.0025, ki=0.0015, kd=1.7, integral_limit=1.0)
        # Reduce P and D slightly, and lower I to reduce overshoot
        #self.lateral_pid = PID(kp=0.0009, ki=0.000, kd=0.14, integral_limit=0.6)
        self.lateral_pid = PID(kp=0.0013, ki=0.0014, kd=0.16, integral_limit=1.0)
        #self.lateral_pid = PID(kp=0.00012, ki=0.00055, kd=0.228, integral_limit=0.6)
        #self.lateral_pid = PID(kp=0.0018, ki=0.0007, kd=0.11, integral_limit=1.2)

        self.car_heading_deg = 255.0  # Set desired heading here
        self.heading_log = 0
        self.car_heading_radians = math.radians(self.car_heading_deg) # in radians
        self.log_roll_cmd = []
        self.log_time = []
        self.log_distance_error = []
        self.log_actual_distance = []
        self.log_altitude = []
        self.log_lateral_offset = []
        self.run_duration = 1500.0  # seconds

        self.base_throttle = 0.55
        self.uav_was_ahead = False

        self.takeoff(self.fixed_altitude)
        time.sleep(2)
        self.warmup_throttle()
        self.start_prius_movement()

        airspeed = self.vehicle.airspeed or 0.0
        airspeed_error = 17.0 - airspeed
        if self.airspeed_pid.ki > 0:
            self.airspeed_pid.integral = (self.base_throttle - self.airspeed_pid.kp * airspeed_error) / self.airspeed_pid.ki

        # Start threads
        self.start_attitude_thread()  # 100 Hz control loop
        #self.start_autotuner_thread()
        self.start_gps_thread()       # 5 Hz GPS sending
        # Start ZN tuner in a separate thread (after control loop is running)
        #threading.Thread(target=self.load_or_run_zn_tuner, daemon=True).start()

    def warmup_throttle(self):
        q_init = euler_to_quaternion(0.0, 0.0, 0.0)
        for _ in range(20):
            self.vehicle._master.mav.set_attitude_target_send(
                0,
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                0b00000100,
                q_init,
                0.0, 0.0, 0.0,
                self.base_throttle
            )
            time.sleep(0.01)

    def load_or_run_zn_tuner(self):
        path = "zn_lateral_gains.csv"
        if os.path.exists(path):
            with open(path, newline='') as f:
                reader = csv.reader(f)
                next(reader)  # skip header
                gains = next(reader)
                kp, ki, kd = map(float, gains[:3])
                self.lateral_pid.set_gains(kp, ki, kd)
                print(f"✅ Loaded tuned gains: Kp={kp}, Ki={ki}, Kd={kd}")
        else:
            print("📈 No saved gains. Running ZN tuner.")
            self.ziegler_nichols_tune_lateral()

    def gps_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude

        # Filtered update
        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon

        # Store previous lat/lon before updating
        self.prius_last_lat = self.last_lat
        self.prius_last_lon = self.last_lon

        self.last_lat = lat
        self.last_lon = lon
    def ziegler_nichols_tune_lateral(self):
        print("🔧 Starting Ziegler-Nichols Tuning for lateral PID (no time limit)...")
        self.lateral_pid.set_gains(0.0, 0.0, 0.0)

        Ku = 0.0
        Tu = 0.0
        test_kp = 0.0001
        self.lateral_pid.ki = 0.0
        self.lateral_pid.kd = 0.0

        while rclpy.ok():
            self.lateral_pid.set_gains(test_kp, 0.0, 0.0)
            print(f"[ZNT] Testing Kp = {test_kp:.6f}")
            self.log_lateral_offset.clear()
            self.log_time.clear()

            time.sleep(200)  # Allow system to respond

            errors = self.log_lateral_offset[-300:]
            times = self.log_time[-300:]
            if len(errors) < 100:
                test_kp += 0.0001
                continue

            # Detect zero-crossings (oscillations)
            zero_crossings = []
            for i in range(1, len(errors)):
                if errors[i - 1] * errors[i] < 0:
                    zero_crossings.append(times[i])

            if len(zero_crossings) >= 6:
                Tu = (zero_crossings[-1] - zero_crossings[-5]) / 5.0
                Ku = test_kp
                print(f"✅ Sustained oscillations at Kp = {Ku:.6f}, Tu = {Tu:.2f}s")
                break

            test_kp += 0.0001
            if test_kp > 0.01:
                print("⚠️ Kp too high without stable oscillations. Aborting.")
                return

        if Ku == 0 or Tu == 0:
            print("❌ Could not detect usable oscillations.")
            return

        # Ziegler-Nichols PID tuning
        Kp = 0.6 * Ku
        Ki = 2 * Kp / Tu
        Kd = Kp * Tu / 8

        self.lateral_pid.set_gains(Kp, Ki, Kd)
        print(f"🎯 Ziegler–Nichols PID set:")
        print(f"    Kp = {Kp:.6f}")
        print(f"    Ki = {Ki:.6f}")
        print(f"    Kd = {Kd:.6f}")
        with open("zn_lateral_gains.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Kp", "Ki", "Kd", "Ku", "Tu"])
            writer.writerow([Kp, Ki, Kd, Ku, Tu])

    def apriltag_callback(self, msg):
        if hasattr(msg, 'id'):
            tag_id = msg.id
        else:
            tag_id = 0  # Default if no ID present

        self.latest_tags[tag_id] = (msg.pose, time.time())

    # def offset_gps(self, lat, lon, bearing_rad, distance_m):
    
    #     """Returns a new GPS point (lat, lon) offset by distance_m *behind* bearing_rad.
    #     """
    #     R = 6378137.0  # Earth radius in meters
    #     bearing = bearing_rad  # Right direction

    #     if bearing < 0 and bearing > 90:
    #         lat_distance_offset =  distance_m * math.sin(bearing)
    #         lon_distance_offset = distance_m * math.cos(bearing)

    #     elif bearing > 90 and bearing < 180:
    #         lat_distance_offset = -distance_m * math.sin(180 - bearing)
    #         lon_distance_offset = distance_m * math.cos(180 - bearing)
        
    #     elif bearing > -90 and bearing < 0:
    #         lat_distance_offset = distance_m * math.cos(-bearing)
    #         lon_distance_offset = distance_m * math.sin(-bearing)
    #     else:
    #         lat_distance_offset = -distance_m * math.cos(180 + bearing)
    #         lon_distance_offset = -distance_m * math.sin(180 + bearing)
        

    #     new_lat = math.asin(
    #         math.sin(math.radians(lat)) * math.cos(lat_distance_offset / R) +
    #         math.cos(math.radians(lat)) * math.sin(lat_distance_offset / R) * math.cos(bearing)
    #     )
    #     new_lon = math.radians(lon) + math.atan2(
    #         math.sin(bearing) * math.sin(lon_distance_offset / R) * math.cos(math.radians(lat)),
    #         math.cos(lon_distance_offset / R) - math.sin(math.radians(lat)) * math.sin(new_lat)
    #     )
    #     return math.degrees(new_lat), math.degrees(new_lon)

    def offset_gps(self, lat, lon, bearing_rad, distance_m):
        """
        Offsets the given lat/lon by distance_m *forward* in the bearing_rad direction.
        Returns new (lat, lon).
        """
        R = 6378137.0  # Earth radius in meters
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)

        d_by_r = distance_m / R

        new_lat = math.asin(
            math.sin(lat1) * math.cos(d_by_r) +
            math.cos(lat1) * math.sin(d_by_r) * math.cos(bearing_rad)
        )

        new_lon = lon1 + math.atan2(
            math.sin(bearing_rad) * math.sin(d_by_r) * math.cos(lat1),
            math.cos(d_by_r) - math.sin(lat1) * math.sin(new_lat)
        )

        return math.degrees(new_lat), math.degrees(new_lon)

    
    def projected_distance_along_heading(self, car_lat, car_lon, car_heading_rad, uav_lat, uav_lon):
        """
        Projects the vector from the car to the UAV onto the car's heading direction.
        Returns positive if the UAV is behind the car, negative if ahead.
        """
        # Approximate flat Earth conversion (good for short distances)
        d_north = geodesic((car_lat, car_lon), (uav_lat, car_lon)).meters
        d_east = geodesic((car_lat, car_lon), (car_lat, uav_lon)).meters
        if uav_lat < car_lat:
            d_north *= -1
        if uav_lon < car_lon:
            d_east *= -1

        heading_x = math.sin(car_heading_rad)
        heading_y = math.cos(car_heading_rad)

        dx = d_east
        dy = d_north

        projection = dx * heading_x + dy * heading_y
        return -projection
    
    def lateral_offset_error(self, car_lat, car_lon, car_heading_rad, uav_lat, uav_lon):
        """
        Compute the perpendicular (left-right) offset from the car's path to the UAV.
        Positive means UAV is to the left of the car's path.
        """
        d_north = geodesic((car_lat, car_lon), (uav_lat, car_lon)).meters
        d_east = geodesic((car_lat, car_lon), (car_lat, uav_lon)).meters
        if uav_lat < car_lat:
            d_north *= -1
        if uav_lon < car_lon:
            d_east *= -1

        # Heading vector
        heading_x = math.sin(car_heading_rad)
        heading_y = math.cos(car_heading_rad)

        # Orthogonal (leftward) vector
        leftward_x = -heading_y
        leftward_y = heading_x

        # Vector from car to UAV
        dx = d_east
        dy = d_north

        # Project UAV vector onto leftward direction
        lateral_error = dx * leftward_x + dy * leftward_y
        return lateral_error
    
    def wrap_angle_deg(self, angle):
        return ((angle + 180) % 360) - 180
    
    def wrap_angle_rad(self, angle):
        """Wrap angle to [-π, π]"""
        return ((angle + math.pi) % (2 * math.pi)) - math.pi 

    def start_gps_thread(self):
        def send_goto_loop():
            start_time = time.time()
            rate = 0.2  # 5 Hz
            while rclpy.ok():
                if self.last_lat is not None:
                    elapsed = time.time() - start_time
                    if elapsed >= 5.0:
                        spoof_lat, spoof_lon = self.offset_gps(self.last_lat, self.last_lon, self.car_heading_radians, 50.0)
                        self.vehicle.simple_goto(LocationGlobalRelative(spoof_lat, spoof_lon, self.fixed_altitude))
                    else:
                        self.vehicle.simple_goto(LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude))

                    # target = LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude)
                    # self.vehicle.simple_goto(target)
                time.sleep(rate)

        thread = threading.Thread(target=send_goto_loop)
        thread.daemon = True
        thread.start()


    def attitude_loop(self):
        log_counter = 0
        while rclpy.ok():
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            elapsed = now - self.start_time

            if self.last_lat is None or self.last_lon is None or (now - self.start_time) < 30.0:
                time.sleep(0.01)
                continue

            self.uav_lat = self.vehicle.location.global_relative_frame.lat
            self.uav_lon = self.vehicle.location.global_relative_frame.lon
            altitude = self.vehicle.location.global_relative_frame.alt or 0.0
            airspeed = self.vehicle.airspeed or 0.0
            now = time.time()
            pose0, t0 = self.latest_tags.get(0, (None, 0.0))
            pose1, t1 = self.latest_tags.get(1, (None, 0.0))
            desired_distance = 10.0 if elapsed < 100.0 else 0.0

            use_pose = None
            tag_source = "GPS"

            valid_time_window = 1.0  # seconds

            # --- Determine best valid AprilTag pose ---
            if now - t0 < valid_time_window:
                use_pose = pose0
                tag_source = "AprilTag ID 0"
            elif now - t1 < valid_time_window:
                use_pose = pose1
                tag_source = "AprilTag ID 1"

            # --- If not fresh, use last one briefly ---
            elif t0 > t1 and (now - t0) < 1.5:
                use_pose = pose0
                tag_source = "Stale AprilTag ID 0"
            elif t1 > t0 and (now - t1) < 1.5:
                use_pose = pose1
                tag_source = "Stale AprilTag ID 1"
            
            if use_pose is not None:
                
                tag_source = "AprilTag ID 0" if now - t0 < 0.5 else "AprilTag ID 1"
                if self.prev_tag_source != tag_source:
                    self.distance_pid.integral = 0.0
                    self.distance_pid.last_error = 0.0
                lateral_error = use_pose.pose.position.x
                altitude_error = self.fixed_altitude - (self.vehicle.location.global_relative_frame.alt or 0.0)
                distance_error = use_pose.pose.position.z - desired_distance
                dist_to_target = use_pose.pose.position.z

            else:
                tag_source = "GPS"
                if self.prius_last_lat is not None and self.prius_last_lon is not None:
                    dist_to_target = geodesic((self.uav_lat, self.uav_lon), (self.last_lat, self.last_lon)).meters

                distance_error = dist_to_target - desired_distance
                lateral_error = self.lateral_offset_error(self.last_lat, self.last_lon, self.car_heading_radians, self.uav_lat, self.uav_lon)
                altitude_error = self.fixed_altitude - (self.vehicle.location.global_relative_frame.alt or 0.0)

            self.prev_dist = dist_to_target

            if elapsed > self.run_duration:
                self.get_logger().info("✅ Run complete. Shutting down...")
                rclpy.shutdown()
                break

            # --- Target airspeed with PID ---
            target_airspeed = 16.0 + self.distance_pid.update(distance_error, dt)  # baseline matching speed

            # --- Final clamp ---
            target_airspeed = max(10.0, min(target_airspeed, 22.0))

            # --- Altitude + Pitch ---
            altitude_error = self.fixed_altitude - altitude
            pitch_cmd = self.altitude_pid.update(altitude_error, dt)
            pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))

            # --- Throttle for airspeed ---
            airspeed_error = target_airspeed - airspeed
            throttle_adjust = self.airspeed_pid.update(airspeed_error, dt)
            throttle_cmd = self.base_throttle + throttle_adjust
            throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

            # --- Lateral Error to Roll Command ---
            lateral_error = self.lateral_offset_error(self.last_lat, self.last_lon, self.car_heading_radians, self.uav_lat, self.uav_lon)
            roll_cmd = self.lateral_pid.update(lateral_error, dt)
            roll_cmd = max(min(roll_cmd, math.radians(10)), math.radians(-10))

            if math.isfinite(altitude):
                elapsed = now - self.start_time
                self.log_time.append(elapsed)
                self.log_distance_error.append(distance_error)
                self.log_actual_distance.append(dist_to_target)
                self.log_altitude.append(altitude)
                self.log_roll_cmd.append(math.degrees(roll_cmd))
                self.log_lateral_offset.append(lateral_error)

            #--- Attitude Command ---#
            q = euler_to_quaternion(roll_cmd, pitch_cmd, 0.0)
            self.vehicle._master.mav.set_attitude_target_send(
                int((now - self.start_time) * 1000),
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                0b00000100,
                q,
                0.0, 0.0, 0.0,
                throttle_cmd
            )

            # self.vehicle._master.mav.command_long_send(
            #     self.vehicle._master.target_system,
            #     self.vehicle._master.target_component,
            #     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            #     0,
            #     0,  # Airspeed
            #     target_airspeed,
            #     -1, 0, 0, 0, 0
            # )

            # # --- Send MAVLINK_MSG_ID_SET_TECS_EXTERNAL_NAV (ID 187) ---
            # self.vehicle._master.mav.command_int_send(
            #     self.vehicle._master.target_system,
            #     self.vehicle._master.target_component,
            #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            #     43008,  # MAV_CMD_SET_TECS_EXTERNAL_NAV
            #     0, 0,     # confirmation
            #     float(target_airspeed),  # ✅ param1: desired EAS in m/s
            #     float(self.fixed_altitude),  # ✅ param2: desired altitude (AGL)
            #     1, 0, 0, 0, 0                         # param3 = enable, rest unused
            # )
            # self.vehicle._master.mav.command_long_send(
            #     self.vehicle._master.target_system,
            #     self.vehicle._master.target_component,
            #     43008,  # MAV_CMD_SET_TECS_EXTERNAL_NAV
            #     0,
            #     float(target_airspeed),  # ✅ param1: desired EAS in m/s
            #     float(self.fixed_altitude),  # ✅ param2: desired altitude (AGL)
            #     1, 0, 0, 0, 0             # param3 = enable, rest unused
            # )

            # --- Compute spoofed forward distance (fake L1) ---
            # car_heading = self.car_heading_radians if hasattr(self, "car_heading_radians") else math.radians(self.target_heading_deg)
            spoofed_forward_distance = 500 # push target 40m ahead for L1 logic
            # # --- Send MAV_CMD_SET_L1_EXTERNAL_NAV ---
            # self.vehicle._master.mav.command_int_send(
            #     self.vehicle._master.target_system,
            #     self.vehicle._master.target_component,
            #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            #     43007,
            #     0, 0,  # current, autocontinue
            #     float(lateral_error),      # param1: crosstrack error (left/right, + = left)
            #     float(spoofed_forward_distance),     # param2: forward distance (m)
            #     1,                         # param3: enable (1), disable (0)
            #     0, 0,                      # param4-5 unused
            #     0, 0                       # param6-7 unused
            # )

            self.send_custom_l1_external_nav(float(lateral_error), float(spoofed_forward_distance), 1)



# Compute spoofed GPS 20m ahead of car for display only
            self.heading_log = self.car_heading_deg
            if hasattr(self, "car_heading_radians"):
                spoof_lat, spoof_lon = self.offset_gps(self.last_lat, self.last_lon, self.car_heading_radians, 30.0)
            else:
                spoof_lat, spoof_lon = None, None
            if log_counter % 10 == 0:
                self.get_logger().info(
                    f"[{tag_source}] Dist: {dist_to_target:.2f}m | TgtAS: {target_airspeed:.2f} | AS: {airspeed:.2f} | "
                    f"Alt: {altitude:.2f} | Throttle: {throttle_cmd:.2f} | Pitch(deg): {math.degrees(pitch_cmd):.2f} | "
                    f"Lateral Offset: {lateral_error:.2f}m | RollCmd: {math.degrees(roll_cmd):.2f}°"
                    f"Bearing Car: {self.heading_log:.2f}° | "
                )
            log_counter += 1
            
            time.sleep(0.01)

    def compute_bearing(self, lat1, lon1, lat2, lon2):
        """
        Compute the bearing from (lat1, lon1) to (lat2, lon2) in radians.
        """
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
        bearing = math.atan2(x, y)
        return bearing  # in radians
    
    def send_custom_l1_external_nav(self, xtrack_error, wp_distance, enable):
        self.vehicle._master.mav.l1_external_nav_send(
            xtrack_error,     # param1: cross-track error (left/right)
            wp_distance,      # param2: distance to WP
            enable            # param3: 1=enable, 0=disable
        )

    def start_attitude_thread(self):
        thread = threading.Thread(target=self.attitude_loop)
        thread.daemon = True
        thread.start()

    def start_autotuner_thread(self):
        thread = threading.Thread(target=self.lateral_pid_autotune_loop)
        thread.daemon = True
        thread.start()

    def lateral_pid_autotune_loop(self):
        kp, ki, kd = self.lateral_pid.kp, self.lateral_pid.ki, self.lateral_pid.kd
        deltas = {"kp": 0.0001, "ki": 0.00005, "kd": 0.001}
        history = []

        start_autotune = time.time()
        max_duration = self.run_duration + 10.0  # 🕒 Run slightly longer than mission
        stale_count = 0
        max_stale_rounds = 3
        min_improvement = 0.01  # Must improve by at least this much

        while rclpy.ok():
            if time.time() - start_autotune > max_duration:
                print("[Autotune] ⏹ Stopping after max duration.")
                break

            time.sleep(10.0)

            if len(self.log_lateral_offset) < 200:
                continue

            # 📈 Score current gain setup
            error = sum(abs(e) for e in self.log_lateral_offset[-200:]) / 200.0
            print(f"[Autotune] Current error: {error:.4f}")
            history.append((kp, ki, kd, error))
            if len(history) > 5:
                history.pop(0)

            best_score = error
            best_gains = (kp, ki, kd)

            for param in ["kp", "ki", "kd"]:
                for direction in [-1, 1]:
                    test_kp = kp + direction * deltas["kp"] if param == "kp" else kp
                    test_ki = ki + direction * deltas["ki"] if param == "ki" else ki
                    test_kd = kd + direction * deltas["kd"] if param == "kd" else kd

                    self.lateral_pid.set_gains(test_kp, test_ki, test_kd)
                    time.sleep(5.0)

                    # 🔁 Score new config
                    score = sum(abs(e) for e in self.log_lateral_offset[-100:]) / 100.0

                    if score < best_score:
                        best_score = score
                        best_gains = (test_kp, test_ki, test_kd)
                        print(f"[TUNE] ✅ Better gain: kp={test_kp}, ki={test_ki}, kd={test_kd} → score={score:.4f}")

            # 📉 Check for improvement
            if best_score < error - min_improvement:
                stale_count = 0
            else:
                stale_count += 1
                print(f"[Autotune] ⚠️ No significant improvement. ({stale_count}/{max_stale_rounds})")

            if stale_count >= max_stale_rounds:
                print("[Autotune] 🛑 No improvement over several rounds. Stopping.")
                break

            # ✅ Apply best gains
            kp, ki, kd = best_gains
            self.lateral_pid.set_gains(kp, ki, kd)
            print(f"[TUNE] Applying: kp={kp}, ki={ki}, kd={kd}")

            # 💾 Log to CSV
            with open("live_autotune_log.csv", "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([time.time(), kp, ki, kd, best_score])


    def speed_profile(self, dist, max_speed=25.0, min_speed=15.0, slowdown_radius=100.0):
        if dist > slowdown_radius:
            return max_speed
        else:
            return min_speed + (max_speed - min_speed) * (dist / slowdown_radius)

    def is_uav_ahead(self, car_lat, car_lon, car_heading_rad, uav_lat, uav_lon):
        """
        Check if UAV is in front of the car (along the heading vector).
        Returns True if UAV is ahead.
        """
        # Convert to flat Earth frame (meters), small distance approximation
        d_north = geodesic((car_lat, car_lon), (uav_lat, car_lon)).meters
        d_east = geodesic((car_lat, car_lon), (car_lat, uav_lon)).meters
        if uav_lat < car_lat:
            d_north *= -1
        if uav_lon < car_lon:
            d_east *= -1

        # Project UAV vector onto heading direction
        dx = d_east
        dy = d_north
        heading_dx = math.sin(car_heading_rad)
        heading_dy = math.cos(car_heading_rad)

        dot_product = dx * heading_dx + dy * heading_dy
        return dot_product > 0  # UAV is in front if projection is positive


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
            altitude = self.vehicle.location.global_relative_frame.alt
            self.get_logger().info(f"Altitude: {altitude:.1f}m")
            if altitude >= target_altitude * 0.95:
                self.get_logger().info("Reached target altitude!")
                break
            time.sleep(1)

    def start_prius_movement(self):
        vel_msg = Twist()
        vel_msg.linear.x = 16.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("Prius moving forward at 16 m/s...")

def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()
    try:
        rclpy.spin(gps_follower)
    except KeyboardInterrupt:
        pass
    finally:
        gps_follower.destroy_node()

        csv_filename = "lateral_roll_data.csv"
        with open(csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Lateral Offset (m)", "Roll Command (deg)", "car heading"])
            for t, offset, roll in zip(gps_follower.log_time, gps_follower.log_lateral_offset, gps_follower.log_roll_cmd):
                writer.writerow([t, offset, roll])
        print(f"✅ CSV saved to: {os.path.abspath(csv_filename)}")

        # --- Plot after node is destroyed ---
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 5))
        plt.plot(gps_follower.log_time, gps_follower.log_actual_distance, label="UAV Distance to Car (m)")
        plt.axhline(10.0, linestyle='--', color='gray', label="Target Distance (10m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Distance (m)")
        plt.title("UAV Following Distance Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        plt.figure(figsize=(10, 5))
        plt.plot(gps_follower.log_time, gps_follower.log_altitude, label="UAV Altitude (m)")
        plt.axhline(10.0, linestyle='--', color='gray', label="Target Altitude")
        plt.xlabel("Time (s)")
        plt.ylabel("Distance (m)")
        plt.title("UAV Altitude Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        plt.figure(figsize=(10, 5))
        plt.plot(gps_follower.log_time, gps_follower.log_lateral_offset, label="Lateral Offset (m)")
        plt.axhline(0.0, linestyle='--', color='gray', label="Target Offset (0 m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Offset (m)")
        plt.title("UAV Lateral Offset Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    main()
