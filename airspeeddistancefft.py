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

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_single_loop')

        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 15)
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        self.fixed_altitude = 20.0
        self.last_lat, self.last_lon = None, None
        self.uav_lat, self.uav_lon = None, None
        self.start_time = time.time()
        self.control_start_time = None
        self.run_duration = 1000.0
        self.last_alt_command_time = 0.0

        # Logging
        self.log_time = []
        self.log_distance_error = []
        self.log_chirp_input = []
        self.log_altitude = []
        self.log_ref_distance = []
        self.log_actual_airspeed = []
        self.log_actual_distance = []
        self.log_horizontal_distance = []

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

    def reference_distance(self, t):
        if t < 200:
            return 45.0
        elif t < 400:
            return 20.0
        elif t < 600:
            return 15.0
        elif t < 800:
            return 30.0
        elif t < 1001:
            return 50.0

    def compute_airspeed_from_error(self, flat_distance, t):
        ref_dist = self.reference_distance(t)
        error = flat_distance - ref_dist

        k_p = 1.0
        base_speed = 16.0  # car speed
        correction = k_p * error
        airspeed_command = base_speed + correction
        airspeed_command = max(14.0, min(20.0, airspeed_command))  # Clamp

        return airspeed_command, ref_dist, error
    
    def compute_y_offset_meters(self, lat1, lon1, lat2, lon2):
        # Y offset (North-South): fixed longitude, vary latitude
        return geodesic((lat1, lon1), (lat2, lon1)).meters * (1 if lat2 > lat1 else -1)

    def start_main_loop(self):
        def loop():
            while rclpy.ok():
                now = time.time()
                elapsed_total = now - self.start_time
                time_since_last_alt_cmd = now - self.last_alt_command_time

                if self.last_lat is None or self.last_lon is None:
                    time.sleep(1/5)
                    continue

                # Goto always
                target = LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude)
                self.vehicle.simple_goto(target)

                # Get UAV state
                uav_loc = self.vehicle.location.global_relative_frame
                self.uav_lat = uav_loc.lat
                self.uav_lon = uav_loc.lon
                altitude = uav_loc.alt or 0.0
                airspeed = self.vehicle.groundspeed or 0.0
                dist_to_target = self.compute_y_offset_meters(self.uav_lat, self.uav_lon, self.last_lat, self.last_lon)

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
                        1,  # Groundspeed
                        target_airspeed,
                        -1, 0, 0, 0, 0
                    )

                    # Logging
                    self.log_time.append(elapsed)
                    self.log_distance_error.append(error)
                    self.log_chirp_input.append(target_airspeed)
                    self.log_altitude.append(altitude)
                    self.log_ref_distance.append(ref_dist)
                    self.log_actual_airspeed.append(airspeed)
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

                time.sleep(1/5)

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
        with open("distance_log5hz.csv", "w", newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time (s)", "Target Distance (m)", "Distance Error (m)", "Altitude (m)", "Airspeed Command (m/s)", "Actual Airspeed (m/s)", "Distance (m)"])
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
