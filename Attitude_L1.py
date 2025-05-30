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
        self.distance_pid = PID(kp=0.1, ki=0.0, kd=0.0, integral_limit=5.0)
        self.lateral_pid = PID(kp=0.0013, ki=0.0014, kd=0.16, integral_limit=1.0)
    
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

    def apriltag_callback(self, msg):
        if hasattr(msg, 'id'):
            tag_id = msg.id
        else:
            tag_id = 0  # Default if no ID present

        self.latest_tags[tag_id] = (msg.pose, time.time())


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
            desired_distance = 10.0 if elapsed < 100.0 else 0.0
            use_pose = None
            tag_source = "GPS"
            valid_time_window = 1.0  # seconds
            stale_window = 1.5       # extended stale fallback

            # --- Collect tag data (pose, timestamp) ---
            tag_data = [(id, *self.latest_tags.get(id, (None, 0.0))) for id in [4, 3, 2, 1, 0]]

            # --- Try valid/fresh detections first ---
            for tag_id, pose, t in tag_data:
                if now - t < valid_time_window:
                    use_pose = pose
                    tag_source = f"AprilTag ID {tag_id}"
                    break

            # --- If no fresh tag, try stale (within stale_window) ---
            if use_pose is None:
                # sort by freshest timestamp first
                tag_data_sorted = sorted(tag_data, key=lambda x: x[2], reverse=True)
                for tag_id, pose, t in tag_data_sorted:
                    if now - t < stale_window:
                        use_pose = pose
                        tag_source = f"Stale AprilTag ID {tag_id}"
                        break

            if use_pose is not None:
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
            pitch_cmd = self.altitude_pid.update(altitude_error, dt)
            pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))

            # --- Throttle for airspeed ---
            airspeed_error = target_airspeed - airspeed
            throttle_adjust = self.airspeed_pid.update(airspeed_error, dt)
            throttle_cmd = self.base_throttle + throttle_adjust
            throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

            # --- Lateral Error to Roll Command ---
            roll_cmd = self.lateral_pid.update(lateral_error, dt)
            roll_cmd = max(min(roll_cmd, math.radians(10)), math.radians(-10))

            
            #Just for logging
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

            #For TECS
            # self.vehicle._master.mav.command_long_send(
            #     self.vehicle._master.target_system,
            #     self.vehicle._master.target_component,
            #     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            #     0,
            #     0,  # Airspeed
            #     target_airspeed,
            #     -1, 0, 0, 0, 0
            # )


            spoofed_forward_distance = 500 # push target 40m ahead for L1 logic

            self.send_custom_l1_external_nav(float(lateral_error), float(spoofed_forward_distance), 1)

            if log_counter % 10 == 0:
                self.get_logger().info(
                    f"[{tag_source}] Dist: {dist_to_target:.2f}m | TgtAS: {target_airspeed:.2f} | AS: {airspeed:.2f} | "
                    f"Alt: {altitude:.2f} | Throttle: {throttle_cmd:.2f} | Pitch(deg): {math.degrees(pitch_cmd):.2f} | "
                    f"Lateral Offset: {lateral_error:.2f}m | RollCmd: {math.degrees(roll_cmd):.2f}°"
                )
            log_counter += 1
            
            time.sleep(0.01)

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
