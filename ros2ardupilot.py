import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        # Create a publisher on the '/ap/cmd_vel' topic
        self.publisher_ = self.create_publisher(Twist, '/ap/cmd_vel', 10)
        # Timer to periodically call the publish method
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz

    def publish_velocity(self):
        # Create a Twist message to send velocity commands
        msg = Twist()
        msg.linear.x = 0.0  # x velocity
        msg.linear.y = 0.0  # y velocity
        msg.linear.z = 20.0  # z velocity
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5  # z angular velocity (rotation around z axis)

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)

    # Shutdown the ROS 2 node
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
