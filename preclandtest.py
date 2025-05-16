import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from dronekit import connect
from pymavlink import mavutil
import time
import threading


class LandingTargetBridge(Node):
    def __init__(self, vehicle):
        super().__init__('landing_target_bridge')
        self.vehicle = vehicle

        self.get_logger().info("DroneKit connected to vehicle")
        self.subscription = self.create_subscription(
            PoseStamped,
            '/apriltag/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /apriltag/pose")

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.get_logger().info(f"LANDING_TARGET → x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")

        # LANDING_TARGET with position fields (MAVLink 2)
        mav_msg = self.vehicle.message_factory.landing_target_encode(
            int(time.time() * 1e6),                  # time_usec
            0,                                       # target_num
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,     # frame: local NED
            0.0,                                     # angle_x (unused)
            0.0,                                     # angle_y (unused)
            0.0,                                     # distance (unused)
            0.0, 0.0,                                # size_x, size_y (unused)
            x, y, z,                                 # position
            [1.0, 0.0, 0.0, 0.0],                    # orientation quaternion (identity)
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,  # type
            1,                                       # position_valid
        )

        self.vehicle.send_mavlink(mav_msg)
        self.vehicle.flush()


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    rclpy.init()
    node = LandingTargetBridge(vehicle)

    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")

    node.destroy_node()
    rclpy.shutdown()
    vehicle.close()


if __name__ == '__main__':
    main()
