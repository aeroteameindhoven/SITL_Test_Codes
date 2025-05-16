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
        super().__init__('gps_follower_goto_speed_after_23s')

        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        self.fixed_altitude = 20.0
        self.last_lat, self.last_lon = None, None
        self.start_time = time.time()
        self.control_started = False
        self.run_duration = 60.0

        self.takeoff(self.fixed_altitude)
        time.sleep(2)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 15)
        self.start_prius_movement()

        self.start_goto_thread()

    def gps_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude

        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon

        self.last_lat = lat
        self.last_lon = lon

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

    def reference_distance(self, t):
        return 40.0 if t < 10.0 else 30.0

    def compute_airspeed_from_error(self, dist, t):
        ref = self.reference_distance(t)
        error = dist - ref
        base_speed = 14.0
        k_p = 1.0
        speed = base_speed + k_p * error
        speed = max(14.0, min(20.0, speed))
        return speed, ref, error

    def start_goto_thread(self):
        def loop():
            while rclpy.ok():
                now = time.time()
                elapsed = now - self.start_time

                if self.last_lat is None or self.last_lon is None:
                    time.sleep(1.0 / 5.0)
                    continue

                # Send GOTO
                target = LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude)
                self.vehicle.simple_goto(target)

                # After 23s, send speed commands too
                if elapsed >= 23.0:
                    loc = self.vehicle.location.global_relative_frame
                    uav_lat, uav_lon = loc.lat, loc.lon
                    dist = geodesic((uav_lat, uav_lon), (self.last_lat, self.last_lon)).meters

                    target_speed, ref, err = self.compute_airspeed_from_error(dist, elapsed)

                    self.vehicle._master.mav.command_long_send(
                        self.vehicle._master.target_system,
                        self.vehicle._master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                        0,
                        0,  # airspeed
                        target_speed,
                        -1, 0, 0, 0, 0
                    )

                    self.get_logger().info(
                        f"[{elapsed:.1f}s] GOTO+SPD | Dist: {dist:.1f} → Ref: {ref} | Speed: {target_speed:.1f}"
                    )
                else:
                    self.get_logger().info(f"[{elapsed:.1f}s] GOTO only")

                if elapsed > self.run_duration:
                    self.get_logger().info("✅ Run complete. Shutting down...")
                    rclpy.shutdown()
                    break

                time.sleep(1.0 / 7.0)

        thread = threading.Thread(target=loop)
        thread.daemon = True
        thread.start()

    def start_prius_movement(self):
        from geometry_msgs.msg import Twist
        vel_msg = Twist()
        vel_msg.linear.x = 16.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("🚗 Prius moving forward at 16 m/s...")


def main(args=None):
    rclpy.init(args=args)
    node = GpsFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
