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

        # ROS Subscribers
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

        # ROS Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag/pose', 10)

        self.bridge = CvBridge()
        self.detector = Detector(families="tag36h11")

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # AprilTag size (meters)
        self.tag_size = 1.5

    def camera_info_callback(self, msg):
        self.get_logger().info("Received camera calibration info.")
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float32)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for camera_info...")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        if self.dist_coeffs is not None and np.any(self.dist_coeffs):
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        tags = self.detector.detect(frame, estimate_tag_pose=True, camera_params=(
            self.camera_matrix[0, 0],  # fx
            self.camera_matrix[1, 1],  # fy
            self.camera_matrix[0, 2],  # cx
            self.camera_matrix[1, 2]   # cy
        ), tag_size=self.tag_size)

        if not tags:
            self.get_logger().warn("No AprilTags detected!")
            return

        for tag in tags:
            self.get_logger().info(f"Detected AprilTag ID: {tag.tag_id}")

            if tag.pose_t is None or tag.pose_R is None:
                self.get_logger().error(f"AprilTag {tag.tag_id} detected, but pose data is None!")
                continue

            # Build PoseStamped in camera frame
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera"

            pose_msg.pose.position.x = tag.pose_t[2][0]
            pose_msg.pose.position.y = tag.pose_t[0][0]
            pose_msg.pose.position.z = tag.pose_t[1][0]

            rmat = np.array(tag.pose_R).reshape(3, 3)
            u, _, vt = np.linalg.svd(rmat)
            rmat = np.dot(u, vt)

            if np.linalg.det(rmat) < 0:
                self.get_logger().error("Invalid rotation matrix detected! Fixing...")
                rmat[:, 2] *= -1

            try:
                q = Rotation.from_matrix(rmat).as_quat()
            except ValueError as e:
                self.get_logger().error(f"Error converting rotation matrix: {e}")
                continue

            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]

            # 🔄 Transform pose from camera → base_link
            transformed_pose = self.transform_to_base_link(pose_msg)

            # ✅ Publish in base_link frame
            self.pose_pub.publish(transformed_pose)

    def transform_to_base_link(self, pose_msg):
        def pose_to_matrix(position, rpy_deg):
            r = Rotation.from_euler('xyz', np.radians(rpy_deg))
            T = np.eye(4)
            T[:3, :3] = r.as_matrix()
            T[:3, 3] = position
            return T

        def pose_stamped_to_matrix(pose_msg):
            pos = pose_msg.pose.position
            ori = pose_msg.pose.orientation
            T = np.eye(4)
            T[:3, 3] = [pos.x, pos.y, pos.z]
            r = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
            T[:3, :3] = r.as_matrix()
            return T

        def matrix_to_pose_stamped(T, frame_id, stamp):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = frame_id
            pose_msg.header.stamp = stamp
            pose_msg.pose.position.x = T[0, 3]
            pose_msg.pose.position.y = T[1, 3]
            pose_msg.pose.position.z = T[2, 3]
            r = Rotation.from_matrix(T[:3, :3])
            q = r.as_quat()
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            return pose_msg

        # Extract transforms from SDF:
        T_mount_to_base = pose_to_matrix([0, 0, -0.2], [0, 0, 0])
        T_camera_to_mount = pose_to_matrix([0, 0, 0], [0, 25, 0])  # camera pitched down

        T_camera_to_base = T_mount_to_base @ T_camera_to_mount
        T_tag_in_camera = pose_stamped_to_matrix(pose_msg)
        T_tag_in_base = T_camera_to_base @ T_tag_in_camera

        return matrix_to_pose_stamped(T_tag_in_base, "base_link", pose_msg.header.stamp)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
