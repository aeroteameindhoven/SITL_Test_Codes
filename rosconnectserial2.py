import rclpy
from rclpy.node import Node
from dronekit import connect
from std_msgs.msg import String

class UAV2Node(Node):
    def __init__(self):
        super().__init__('uav2_node')
        self.publisher = self.create_publisher(String, 'uav2/location', 10)
        self.serial_path = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A906H62E-if00-port0'
        self.connect_vehicle()

    def connect_vehicle(self):
        try:
            self.get_logger().info(f"[UAV2] Connecting to {self.serial_path}...")
            self.vehicle = connect(self.serial_path, wait_ready=True, baud=57600)
            firmware = str(self.vehicle.version)

            if "Plane" not in firmware:
                self.get_logger().error(f"[UAV2] Not ArduPlane! Found: {firmware}")
                return

            self.get_logger().info(f"[UAV2] Connected with firmware: {firmware}")
            self.create_timer(1.0, self.publish_location)

        except Exception as e:
            self.get_logger().error(f"[UAV2] Connection error: {e}")

    def publish_location(self):
        loc = self.vehicle.location.global_frame
        msg = String()
        msg.data = f"Lat: {loc.lat}, Lon: {loc.lon}, Alt: {loc.alt}"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UAV2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
