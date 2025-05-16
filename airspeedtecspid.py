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
from apriltag_interfaces.msg import TagPoseStamped


class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_single_loop')

        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 15)
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        self.fixed_altitude = 10.0
        self.last_lat, self.last_lon = None, None
        self.uav_lat, self.uav_lon = None, None
        self.start_time = time.time()
        self.control_start_time = None
        self.run_duration = 400.0
        self.last_time = time.time()
        self.pid_integral = 0.0
        self.prev_error = 0.0
        self.last_target_airspeed = 14.0  # For rate-limiting
        self.prev_error = 0.0            # For derivative control
        self.smoothed_target_airspeed = None

        # Logging
        self.log_time = []
        self.log_distance_error = []
        self.log_chirp_input = []
        self.log_altitude = []
        self.log_ref_distance = []
        self.log_actual_airspeed = []
        self.log_actual_distance = []
        self.log_horizontal_distance = []
        self.visible_tags = {}  # tag_id -> geometry_msgs/Pose
        self.last_tag_time = 0
        self.tag_timeout = 2.0  # seconds to keep a tag considered "active"
        self.create_subscription(TagPoseStamped, '/apriltag/pose_in_base', self.tag_callback, 10)

        self.takeoff(self.fixed_altitude)
        time.sleep(2)
        self.start_prius_movement()
        self.start_main_loop()

    def gps_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude
        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon
        self.last_lat = lat
        self.last_lon = lon
    
    def tag_callback(self, msg: TagPoseStamped):
        self.visible_tags[msg.tag_id] = msg.pose
        self.last_tag_time = time.time()

    def reference_distance(self, t):
        if t < 100:
            return 20.0
        elif t < 200:
            return 15.0
        elif t < 301:
            return 12.0
        elif t < 400:
            return 10.0

    def compute_airspeed_from_error(self, distance, t):
        ref_dist = self.reference_distance(t)
        error = distance - ref_dist  # ✅ Signed error to handle overshoot

        dt = max(0.001, time.time() - self.last_time)

        # Updated PID (more damped, less aggressive derivative)
        k_p = 1
        k_i = 0
        k_d = 0
        
        # Integral term
        self.pid_integral += error * dt

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        correction = k_p * error + k_i * self.pid_integral + k_d * derivative

        base_speed = 16.0
        raw_target_airspeed = base_speed + correction
        raw_target_airspeed = max(14.0, min(20.0, raw_target_airspeed))

        self.last_time = time.time()
        return raw_target_airspeed, ref_dist, error

    def start_main_loop(self):
        def loop():
            while rclpy.ok():
                now = time.time()
                elapsed_total = now - self.start_time

                if self.last_lat is None or self.last_lon is None:
                    time.sleep(1.0 / 5.0)
                    continue

                # Goto always
                current_time = time.time()
                use_tag = False

                # Prioritize tag 0 > tag 1
                tag_pose = None
                if (current_time - self.last_tag_time) < self.tag_timeout:
                    if 0 in self.visible_tags:
                        tag_pose = self.visible_tags[0]
                        use_tag = True
                    elif 1 in self.visible_tags:
                        tag_pose = self.visible_tags[1]
                        use_tag = True

                if use_tag and self.uav_lat is not None and self.uav_lon is not None:
                    # Use tag-relative offset: x (east), y (north)
                    x_offset = tag_pose.position.z
                    y_offset = tag_pose.position.x

                    d_lat = y_offset / 111320.0
                    d_lon = x_offset / (40075000 * math.cos(math.radians(self.uav_lat)) / 360.0)

                    tag_target_lat = self.uav_lat + d_lat
                    tag_target_lon = self.uav_lon + d_lon

                    target = LocationGlobalRelative(tag_target_lat, tag_target_lon, self.fixed_altitude)
                    self.get_logger().info(f"🔵 Using Apriltag ID {'0' if 0 in self.visible_tags else '1'} at offset x={x_offset:.2f}, y={y_offset:.2f}")
                else:
                    target = LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude)
                    self.get_logger().info("🟡 Using GPS target")

                self.vehicle.simple_goto(target)

                # Get UAV state
                uav_loc = self.vehicle.location.global_relative_frame
                self.uav_lat = uav_loc.lat
                self.uav_lon = uav_loc.lon
                altitude = uav_loc.alt or 0.0
                airspeed = self.vehicle.airspeed or 0.0
                dist_to_target = geodesic((self.uav_lat, self.uav_lon), (self.last_lat, self.last_lon)).meters

                # Start control after 23s
                if self.control_start_time is None and elapsed_total >= 23.0:
                    self.control_start_time = now

                if self.control_start_time is not None:
                    elapsed = now - self.control_start_time
                    target_airspeed, ref_dist, error = self.compute_airspeed_from_error(dist_to_target, elapsed)

                    # Send DO_CHANGE_SPEED
                    self.vehicle._master.mav.command_long_send(
                        self.vehicle._master.target_system,
                        self.vehicle._master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                        0,
                        0,  # Airspeed
                        target_airspeed,
                        -1, 0, 0, 0, 0
                    )

                    # Logging
                    self.log_time.append(elapsed)
                    self.log_distance_error.append(error)
                    self.log_chirp_input.append(target_airspeed)
                    self.log_altitude.append(altitude)
                    self.log_ref_distance.append(ref_dist)
                    if self.log_actual_airspeed:
                        smoothed_airspeed = 0.8 * self.log_actual_airspeed[-1] + 0.2 * airspeed
                    else:
                        smoothed_airspeed = airspeed
                    self.log_actual_airspeed.append(smoothed_airspeed)
                    self.log_actual_distance.append(dist_to_target)

                    self.get_logger().info(
                        f"[{elapsed:.1f}s] Dist: {dist_to_target:.1f} | Ref: {ref_dist:.1f} | Target AS: {target_airspeed:.1f} | "
                        f"AS: {airspeed:.1f} | Alt: {altitude:.1f}"
                    )

                    if elapsed > self.run_duration:
                        self.get_logger().info("✅ Run complete. Shutting down...")
                        rclpy.shutdown()
                        break
                else:
                    self.get_logger().info(f"[{elapsed_total:.1f}s] GOTO only")

                time.sleep(1.0 / 5.0)

        thread = threading.Thread(target=loop)
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
        self.get_logger().info("🚗 Prius moving forward at 16 m/s...")

def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()
    try:
        rclpy.spin(gps_follower)
    except KeyboardInterrupt:
        pass
    finally:
        gps_follower.destroy_node()

        # Save data to CSV
        with open("distance_log2.csv", "w", newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time (s)", "Target Distance (m)", "Distance Error (m)", "Altitude (m)", "Airspeed Command (m/s)", "Actual Airspeed (m/s), Horisontal Distance (m)"])
            for t, ref_d, d, alt, a_cmd, a_act, d_actual in zip(
                gps_follower.log_time,
                gps_follower.log_ref_distance,
                gps_follower.log_distance_error,
                gps_follower.log_altitude,
                gps_follower.log_chirp_input,
                gps_follower.log_actual_airspeed,
                gps_follower.log_actual_distance
            ):
                writer.writerow([t, ref_d, d, alt, a_cmd, a_act, d_actual])

        # Plot
        plt.figure(figsize=(12, 6))

        # Altitude
        plt.subplot(3, 1, 1)
        plt.plot(gps_follower.log_time, gps_follower.log_altitude, label="Altitude (m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude")
        plt.title("Altitude Tracking")
        plt.grid(True)
        plt.legend()

        # Distance
        plt.subplot(3, 1, 2)
        plt.plot(gps_follower.log_time, gps_follower.log_actual_distance, label="Actual Distance", color='orange')
        plt.plot(gps_follower.log_time, gps_follower.log_ref_distance, label="Target Distance", linestyle='--', color='blue')
        plt.xlabel("Time (s)")
        plt.ylabel("Distance (m)")
        plt.title("Distance Tracking")
        plt.grid(True)
        plt.legend()

        # Airspeed
        plt.subplot(3, 1, 3)
        plt.plot(gps_follower.log_time, gps_follower.log_actual_airspeed, label="Actual Airspeed", color='orange')
        plt.plot(gps_follower.log_time, gps_follower.log_chirp_input, label="Target Airspeed", linestyle='--', color='blue')
        plt.xlabel("Time (s)")
        plt.ylabel("Airspeed (m/s)")
        plt.title("Airspeed Tracking")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    main()
