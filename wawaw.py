import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # ✅ ROS Subscribers
        self.image_sub = self.create_subscription(
            Image, 
            '/gimbal_camera/image',  
            self.image_callback, 
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/gimbal_camera/camera_info',
            self.camera_info_callback,
            10
        )

        # ✅ ROS Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag/pose', 10)

        self.bridge = CvBridge()
        self.detector = Detector(families="tag36h11")

        # ✅ Camera Calibration (from /camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None  # Default None until received

        # ✅ AprilTag Size in meters (2m)
        self.tag_size = 1.5

    def camera_info_callback(self, msg):
        """Extracts camera intrinsic matrix from /camera_info topic."""
        self.get_logger().info("Received camera calibration info.")

        # ✅ Convert camera info to NumPy array
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float32)

        self.get_logger().info(f"Camera Matrix: \n{self.camera_matrix}")
        self.get_logger().info(f"Distortion Coefficients: {self.dist_coeffs}")

    def image_callback(self, msg):
        """Detect AprilTags once camera info is available."""
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for camera_info...")
            return

        # ✅ Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        # ✅ Undistort image if distortion coefficients are provided
        if self.dist_coeffs is not None and np.any(self.dist_coeffs):
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        # ✅ Detect AprilTags
        tags = self.detector.detect(frame, estimate_tag_pose=True, camera_params=(
            self.camera_matrix[0, 0],  # fx
            self.camera_matrix[1, 1],  # fy
            self.camera_matrix[0, 2],  # cx
            self.camera_matrix[1, 2]   # cy
        ), tag_size=self.tag_size)

        if tags:
            for tag in tags:
                self.get_logger().info(f"Detected AprilTag ID: {tag.tag_id}")

                # ✅ Check if pose estimation succeeded
                if tag.pose_t is None:
                    self.get_logger().error(f"AprilTag {tag.tag_id} detected, but pose_t is None!")
                    continue  # Skip this tag

                # ✅ Prepare PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera"

                # ✅ Position assignment remains as you intended
                pose_msg.pose.position.x = tag.pose_t[2][0]
                pose_msg.pose.position.y = tag.pose_t[0][0]
                pose_msg.pose.position.z = tag.pose_t[1][0]

                # ✅ Convert Rotation Matrix to Quaternion using SciPy
                rmat = np.array(tag.pose_R).reshape(3, 3).T  # Transpose to match SciPy format
                q = Rotation.from_matrix(rmat).as_quat()  # Returns [x, y, z, w]

                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]

                # ✅ Publish detected tag pose
                self.pose_pub.publish(pose_msg)
        else:
            self.get_logger().warn("No AprilTags detected!")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
