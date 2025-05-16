import rclpy
from rclpy.node import Node
from dronekit import connect
from std_msgs.msg import String
from geopy.distance import geodesic
import csv
import time
import os

class UAV2DistanceLoggerNode(Node):
    def __init__(self):
        super().__init__('uav2_distance_logger_node')
        self.serial_path = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A906H62E-if00-port0'
        self.vehicle = None
        self.csv_path = 'uav2_distance_log.csv'
        self.start_time = time.time()

        self.connect_vehicle()
        self.setup_csv()

    def connect_vehicle(self):
        try:
            self.get_logger().info(f"[UAV2] Connecting to {self.serial_path}...")
            self.vehicle = connect(self.serial_path, wait_ready=True, baud=57600)
            self.get_logger().info(f"[UAV2] Connected! Firmware: {self.vehicle.version}")

            self.subscription = self.create_subscription(
                String,
                'uav1/location',
                self.location_callback,
                10
            )
            self.get_logger().info("[UAV2] Subscribed to uav1/location")
        except Exception as e:
            self.get_logger().error(f"[UAV2] Connection error: {e}")

    def setup_csv(self):
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['timestamp', 'horizontal_distance'])

    def location_callback(self, msg: String):
        try:
            parts = msg.data.strip().split(',')
            lat = float(parts[0].split(':')[1].strip())
            lon = float(parts[1].split(':')[1].strip())

            current_loc = self.vehicle.location.global_relative_frame
            if current_loc.lat is None or current_loc.lon is None:
                self.get_logger().warn("[UAV2] Current GPS not available.")
                return

            uav_lat = current_loc.lat
            uav_lon = current_loc.lon

            horizontal_distance = geodesic((uav_lat, uav_lon), (lat, lon)).meters
            timestamp = time.time() - self.start_time

            self.get_logger().info(f"[UAV2] Time: {timestamp:.1f}s | Dist: {horizontal_distance:.2f} m")

            with open(self.csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, horizontal_distance])

        except Exception as e:
            self.get_logger().error(f"[UAV2] Error parsing location: {e}")
            self.get_logger().error(f"[UAV2] Raw msg: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = UAV2DistanceLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
