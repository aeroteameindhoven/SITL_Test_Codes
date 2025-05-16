import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_dronekit')

        # Connect to ArduPilot SITL using DroneKit
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        # ROS 2 Publisher to control Prius velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 15)
        time.sleep(5)

        # ROS 2 Subscriber to Prius GPS topic
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        # Fixed target altitude (UAV always stays at 10m)
        self.fixed_altitude = 10.0

        # Last known position (for smoothing)
        self.last_lat, self.last_lon = None, None

        # Record start time
        self.start_time = time.time()

        # Take off UAV and move Prius
        self.takeoff(self.fixed_altitude)
        time.sleep(2)  # Wait before starting Prius
        self.start_prius_movement()

    def send_gps_target(self, lat, lon):
        """ Command the UAV to move to target GPS coordinates with a fixed altitude of 10m """
        if self.last_lat is not None:
            # Apply basic smoothing using a weighted average (Low-pass filter)
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon

        self.last_lat, self.last_lon = lat, lon

        # Ensure the vehicle is in GUIDED mode
        if self.vehicle.mode != VehicleMode("GUIDED"):
            self.get_logger().warn("UAV is not in GUIDED mode. Switching to GUIDED mode.")
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)

        # Command to new location
        target_location = LocationGlobalRelative(lat, lon, self.fixed_altitude)
        self.get_logger().info(f"Moving UAV to: {lat}, {lon}, Alt: {self.fixed_altitude}m")
        self.vehicle.simple_goto(target_location)
        time.sleep(0.5)

    def gps_callback(self, msg):
        """ Callback when new GPS data is received from Prius """
        if time.time() - self.start_time < 5.0:
            self.get_logger().info("Waiting before UAV starts following the Prius...")
            return  # Ignore updates for first 5 seconds
        lat, lon = msg.latitude, msg.longitude
        self.send_gps_target(lat, lon)

    def takeoff(self, target_altitude):
        """ Arms and takes off the UAV to 10m altitude """
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
        """ Sends a command to move the Prius forward at 16 m/s """
        vel_msg = Twist()
        vel_msg.linear.x = 16.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("Prius moving forward at 16 m/s...")

def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()
    rclpy.spin(gps_follower)
    gps_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
