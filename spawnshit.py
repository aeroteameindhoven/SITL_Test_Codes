import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Marker
from apriltag_interfaces.msg import TagPoseStamped  # Replace with your actual package

class TagVisualizer(Node):
    def __init__(self):
        super().__init__('tag_gz_marker_visualizer')

        self.sub = self.create_subscription(
            TagPoseStamped,
            '/apriltag/pose_in_base',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            Marker,
            '/gz/markers',
            10
        )

    def cb(self, msg):
        marker = Marker()
        marker.id = msg.id
        marker.action = Marker.ADD_MODIFY
        marker.type = Marker.ARROW
        marker.pose = msg.pose.pose

        # Set scale (length, width, height)
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Set color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TagVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
