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

class Rate:
    def __init__(self, hz):
        self.period = 1.0 / hz
        self.last_time = time.time()

    def sleep(self):
        now = time.time()
        elapsed = now - self.last_time
        sleep_time = self.period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        self.last_time = time.time()

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
        self.last_time = time.time()
        self.start_time = self.last_time
        self.latest_tags = {}

        self.prius_last_lat = None
        self.prius_last_lon = None

        # PID controllers
        self.airspeed_pid = PID(kp=0.08, ki=0.04, kd=0.0, integral_limit=5.0)
        self.altitude_pid = PID(kp=0.052, ki=0.0095, kd=0.25, integral_limit=5.5)
        #self.altitude_pid = PID(kp=0.065, ki=0.01, kd=0.05, integral_limit=4.5)
        self.distance_pid = PID(kp=1.0, ki=0.0, kd=0.0, integral_limit=5.0)
        #self.lateral_pid = PID(kp=0.0011, ki=0.0006, kd=0.13, integral_limit=2.0)
        #self.lateral_pid = PID(kp=0.0025, ki=0.0015, kd=1.7, integral_limit=1.0)
        # Reduce P and D slightly, and lower I to reduce overshoot
        #self.lateral_pid = PID(kp=0.0009, ki=0.000, kd=0.14, integral_limit=0.6)
        self.lateral_pid = PID(kp=0.0013, ki=0.0014, kd=0.16, integral_limit=1.0)
        #self.lateral_pid = PID(kp=0.00012, ki=0.00055, kd=0.228, integral_limit=0.6)
        #self.lateral_pid = PID(kp=0.0018, ki=0.0007, kd=0.11, integral_limit=1.2)

        self.log_roll_cmd = []
        self.log_time = []
        self.log_distance_error = []
        self.log_actual_distance = []
        self.log_altitude = []
        self.log_lateral_offset = []
        self.run_duration = 1500.0  # seconds
        self.attitude_rate = 0.01  # seconds
        self.gps_rate = 0.2  # seconds

        self.base_throttle = 550

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
            time.sleep(self.attitude_rate)

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
    

    def start_gps_thread(self):
        def send_goto_loop():
            while rclpy.ok():
                if self.last_lat is not None:
                    target = LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude)
                    self.vehicle.simple_goto(target)
                time.sleep(self.gps_rate)

        thread = threading.Thread(target=send_goto_loop)
        thread.daemon = True
        thread.start()

    def attitude_loop(self):
        log_counter = 0
        dt = self.attitude_rate
        rate = Rate(1.0 / dt)

        while rclpy.ok():
            now = time.time()
            elapsed = now - self.start_time

            if self.last_lat is None or self.last_lon is None or elapsed < 50.0:
                rate.sleep()
                continue

            self.uav_lat = self.vehicle.location.global_relative_frame.lat
            self.uav_lon = self.vehicle.location.global_relative_frame.lon
            altitude = self.vehicle.location.global_relative_frame.alt or 0.0
            airspeed = self.vehicle.airspeed or 0.0
            now = time.time()
            pose0, t0 = self.latest_tags.get(0, (None, 0.0))
            pose1, t1 = self.latest_tags.get(1, (None, 0.0))
            desired_distance = 15.00

            use_pose = None
            tag_source = "GPS"

            if now - t0 < 0.5:
                use_pose = pose0
                tag_source = "AprilTag ID 0"
            elif now - t1 < 0.5:
                use_pose = pose1
                tag_source = "AprilTag ID 1"

            if use_pose is not None:
                lateral_error = use_pose.pose.position.x # this is positive if to the right and negative if to the left
                altitude_error = self.fixed_altitude - (self.vehicle.location.global_relative_frame.alt or 0.0) # Actual Uav coordinate is y for lateral error, z for distance, x for altitude
                distance_error = use_pose.pose.position.z - desired_distance 
                dist_to_target = use_pose.pose.position.z

            else:
                tag_source = "GPS"
                if self.prius_last_lat is not None and self.prius_last_lon is not None:
                    car_move_dist = geodesic((self.prius_last_lat, self.prius_last_lon), (self.last_lat, self.last_lon)).meters
                    if car_move_dist > 1.0:
                        raw_car_heading = self.compute_bearing(
                            self.prius_last_lat, self.prius_last_lon,
                            self.last_lat, self.last_lon
                        )
                        if hasattr(self, "last_car_heading"):
                            alpha = 0.9
                            car_heading = alpha * self.last_car_heading + (1 - alpha) * raw_car_heading
                        else:
                            car_heading = raw_car_heading
                        self.last_car_heading = car_heading
                    else:
                        car_heading = self.last_car_heading

                    dist_to_target = self.projected_distance_along_heading(
                        self.last_lat, self.last_lon, car_heading,
                        self.uav_lat, self.uav_lon
                    )
                else:
                    dist_to_target = geodesic((self.uav_lat, self.uav_lon), (self.last_lat, self.last_lon)).meters

                distance_error = dist_to_target - desired_distance
                lateral_error = self.lateral_offset_error(self.last_lat, self.last_lon, car_heading, self.uav_lat, self.uav_lon)
                altitude_error = self.fixed_altitude - (self.vehicle.location.global_relative_frame.alt or 0.0)

            self.prev_dist = dist_to_target

            if elapsed > self.run_duration:
                self.get_logger().info("✅ Run complete. Shutting down...")
                rclpy.shutdown()
                break

            # --- Target airspeed with PID ---
            target_airspeed = 16.0 + self.distance_pid.update(distance_error, dt)  # baseline matching speed

            # --- Final clamp ---
            target_airspeed = max(14.0, min(target_airspeed, 22.0))

            # --- Altitude + Pitch ---
            altitude_error = self.fixed_altitude - altitude
            pitch_cmd = self.altitude_pid.update(altitude_error, dt)
            pitch_cmd = max(min(pitch_cmd, 500), -500)
            pitch_cmd = round(pitch_cmd)

            # --- Throttle for airspeed ---
            airspeed_error = target_airspeed - airspeed
            throttle_adjust = self.airspeed_pid.update(airspeed_error, dt)
            throttle_cmd = self.base_throttle + throttle_adjust
            throttle_cmd = max(0, min(throttle_cmd, 1000))
            throttle_cmd = round(throttle_cmd)

            # --- Lateral Error to Roll Command ---
            lateral_error = self.lateral_offset_error(self.last_lat, self.last_lon, car_heading, self.uav_lat, self.uav_lon)
            roll_cmd = self.lateral_pid.update(lateral_error, dt)
            roll_cmd = max(min(roll_cmd, 500), -500)
            roll_cmd = round(roll_cmd)

            if math.isfinite(altitude):
                elapsed = now - self.start_time
                self.log_time.append(elapsed)
                self.log_distance_error.append(distance_error)
                self.log_actual_distance.append(dist_to_target)
                self.log_altitude.append(altitude)
                self.log_roll_cmd.append(math.degrees(roll_cmd))
                self.log_lateral_offset.append(lateral_error)

            # --- Attitude Command ---
            #q = euler_to_quaternion(roll_cmd, pitch_cmd, 0.0)
            self.vehicle._master.mav.manual_control_send(
                self.vehicle._master.target_system,
                pitch_cmd,
                roll_cmd,
                throttle_cmd,
                0,
                0
            )

            if log_counter % 10 == 0:
                self.get_logger().info(
                    f"[{tag_source}] Dist: {dist_to_target:.2f}m | TgtAS: {target_airspeed:.2f} | AS: {airspeed:.2f} | "
                    f"Alt: {altitude:.2f} | Throttle: {throttle_cmd:.2f} | Pitch(deg): {math.degrees(pitch_cmd):.2f} | "
                    f"Lateral Offset: {lateral_error:.2f}m | RollCmd: {math.degrees(roll_cmd):.2f}°"
                )
            log_counter += 1
            rate.sleep()

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

    def start_attitude_thread(self):
        thread = threading.Thread(target=self.attitude_loop)
        thread.daemon = True
        thread.start()

    def force_clear_rc_overrides(self):
        self.get_logger().info("🚨 Forcing RC override clear via MAVLink...")
        self.vehicle._master.mav.rc_channels_override_send(
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0  # All 8 RC channels set to 0 = no override
        )
        time.sleep(1)
        self.get_logger().info("✅ RC override force-clear sent.")

    def takeoff(self, target_altitude):
        self.get_logger().info("Setting mode to FBWA for manual takeoff...")
        self.vehicle.mode = VehicleMode("FBWA")
        time.sleep(3)

        self.get_logger().info("Arming UAV...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            self.get_logger().info("Waiting for UAV to arm...")
            time.sleep(1)

        self.get_logger().info("Sending RC3 = 1800 (Throttle) to initiate takeoff for 10 seconds...")
        for i in range(100):  # 100 x 0.1s = 10 seconds
            self.vehicle.channels.overrides['3'] = 1800
            if i % 10 == 0:
                self.get_logger().info(f"Throttle override active... {i/10:.1f}s")
            time.sleep(0.1)

        # Check that override was removed
        if self.vehicle.channels.overrides:
            self.get_logger().warn(f"⚠️ RC overrides still present: {self.vehicle.channels.overrides}")
        else:
            self.get_logger().info("✅ RC overrides cleared.")

        self.get_logger().info("Switching to GUIDED mode for autopilot control...")
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(2)

        self.get_logger().info("Waiting for GUIDED mode...")
        while self.vehicle.mode.name != 'GUIDED':
            self.get_logger().info("Still waiting for GUIDED mode...")
            time.sleep(1)

    def start_prius_movement(self):
        vel_msg = Twist()
        vel_msg.linear.x = 16.0 #constant car speed
        self.cmd_vel_pub.publish(vel_msg)
        self.force_clear_rc_overrides()
        self.get_logger().info("Clearing RC overrides...")
        self.vehicle.channels.overrides['3'] = None  # Explicitly clear just RC3
        time.sleep(1)

        # Optional: force-clear all overrides
        self.vehicle.channels.overrides = {}
        time.sleep(1)
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
            writer.writerow(["Time (s)", "Lateral Offset (m)", "Roll Command (deg)"]) #CSV column headers
            for t, offset, roll in zip(gps_follower.log_time, gps_follower.log_lateral_offset, gps_follower.log_roll_cmd): #Define what goes in CSV
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