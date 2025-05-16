import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from dronekit import connect
import time

class GpsMavlinkBroadcaster(Node):
    def __init__(self):
        super().__init__('gps_mavlink_broadcaster')

        # Connect to ArduPilot
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        # Default velocity values (in cm/s)
        self.vx = 0
        self.vy = 0
        self.vz = 0

        # Subscribe to Prius GPS topic
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        # Subscribe to Prius velocity topic
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Listening for /prius/gps and /cmd_vel messages...")

    def cmd_vel_callback(self, msg):
        # Convert m/s to cm/s
        self.vx = int(msg.linear.x * 100)
        self.vy = int(msg.linear.y * 100)
        self.vz = int(msg.linear.z * 100)
        self.get_logger().info(f"Updated velocity: vx={self.vx} cm/s, vy={self.vy} cm/s, vz={self.vz} cm/s")

    def gps_callback(self, msg):
        lat_int = int(msg.latitude * 1e7)
        lon_int = int(msg.longitude * 1e7)
        alt_mm = int(msg.altitude * 1000)  # Fixed altitude for ship landing

        self.get_logger().info(
            f"Sending MAVLink GLOBAL_POSITION_INT → lat={lat_int}, lon={lon_int}, alt={alt_mm}, vx={self.vx}, vy={self.vy}, vz={self.vz}"
        )

        msg_mav = self.vehicle.message_factory.global_position_int_encode(
            int(time.monotonic() * 1e3),  # time_boot_ms
            lat_int,
            lon_int,
            alt_mm,
            alt_mm,
            self.vx,
            self.vy,
            self.vz,
            0  # hdg unknown
        )

        self.vehicle.send_mavlink(msg_mav)
        self.vehicle.flush()

def main(args=None):
    rclpy.init(args=args)
    node = GpsMavlinkBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
