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
        self.run_duration = 600.0

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
        if t < 75:
            return 20.0
        elif t < 140:
            return 15.0
        elif t < 190:
            return 17.0
        elif t < 250:
            return 25.0
        elif t < 310:
            return 27.0
        elif t < 370:
            return 40.0
        elif t < 460:
            return 35.0
        elif t < 601:
            return 50.0

    def compute_airspeed_from_error(self, distance, t):
        # Step input logic
        if t < 30:
            target_airspeed = 12.0
        elif t < 60:
            target_airspeed = 14.0
        elif t < 90:
            target_airspeed = 13.0
        elif t < 120:
            target_airspeed = 16.0
        else:
            target_airspeed = 15.0

        ref_dist = distance  # still log actual distance
        error = 0.0  # no error needed

        return target_airspeed, ref_dist, error

    def start_main_loop(self):
        def loop():
            while rclpy.ok():
                now = time.time()
                elapsed_total = now - self.start_time

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
        with open("distance_logairspeed5hz.csv", "w", newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time (s)", "Airspeed Input (m/s)", "Distance Output (m)", "Altitude (m)", "Actual Airspeed (m/s)"])
            for t, a_cmd, d_actual, alt, a_act in zip(
                gps_follower.log_time,
                gps_follower.log_chirp_input,
                gps_follower.log_actual_distance,
                gps_follower.log_altitude,
                gps_follower.log_actual_airspeed
            ):
                writer.writerow([t, a_cmd, d_actual, alt, a_act])

        plt.figure(figsize=(12, 6))

        # Distance (output)
        plt.subplot(2, 1, 1)
        plt.plot(gps_follower.log_time, gps_follower.log_actual_distance, label="Distance to Car", color='green')
        plt.xlabel("Time (s)")
        plt.ylabel("Distance (m)")
        plt.title("System Output: Distance Response")
        plt.grid(True)
        plt.legend()

        # Airspeed (input)
        plt.subplot(2, 1, 2)
        plt.plot(gps_follower.log_time, gps_follower.log_chirp_input, label="Commanded Airspeed", linestyle='--', color='blue')
        plt.plot(gps_follower.log_time, gps_follower.log_actual_airspeed, label="Actual Airspeed", color='orange')
        plt.xlabel("Time (s)")
        plt.ylabel("Airspeed (m/s)")
        plt.title("System Input: Step Airspeed Commands")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    main()
