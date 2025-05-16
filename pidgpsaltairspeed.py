import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_to_landing_controller')

        # Connect to ArduPilot via DroneKit
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        # ROS 2 publishers/subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)
        self.create_subscription(PoseStamped, '/apriltag/pose', self.pose_callback, 10)

        # State
        self.fixed_altitude = 10.0           # Starting altitude
        self.current_z = self.fixed_altitude # Current target altitude
        self.following = False
        self.landing_active = False
        self.init_pose_recorded = False

        self.vmin = 15.0
        self.v0 = 20.0
        self.gps_follow_speed = 18.0

        # Start sequence
        self.takeoff(self.fixed_altitude)
        time.sleep(2)
        self.start_prius_movement()
        self.timer = self.create_timer(0.0, self.start_following)
        self.shutdown_timer = self.create_timer(23.0, self.activate_landing)

    def takeoff(self, target_altitude):
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            self.get_logger().info("Waiting for UAV to arm...")
            time.sleep(1)
        self.get_logger().info(f"Taking off to {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                self.get_logger().info(f"Reached takeoff altitude: {alt:.2f}m")
                break
            time.sleep(1)

    def start_prius_movement(self):
        vel_msg = Twist()
        vel_msg.linear.x = 16.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("Prius moving forward at 16 m/s.")

    def start_following(self):
        self.following = True
        self.destroy_timer(self.timer)
        self.get_logger().info("✅ UAV is now following Prius via GPS")

    def gps_callback(self, msg: NavSatFix):
        if not self.following and not self.landing_active:
            return

        lat, lon = msg.latitude, msg.longitude
        altitude = self.fixed_altitude if not self.landing_active else self.current_z
        target_location = LocationGlobalRelative(lat, lon, altitude)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.simple_goto(target_location)

        if not self.landing_active:
            speed_msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                178,
                0,
                1,
                self.gps_follow_speed,
                -1, 0, 0, 0, 0
            )
            self.vehicle.send_mavlink(speed_msg)

        time.sleep(0.5)

    def activate_landing(self):
        self.get_logger().info("🛬 Switching to AprilTag-based landing mode")
        self.following = False
        self.landing_active = True

    def generate_descent_trajectory(self, total_time=15.0, dt=0.5):
        times = np.arange(0, total_time + dt, dt)
        altitudes = self.fixed_altitude * 0.5 * (1 + np.cos(np.pi * times / total_time))
        return times, altitudes

    def pose_callback(self, msg: PoseStamped):
        if not self.landing_active:
            return

        x_rel = msg.pose.position.x
        z_rel = msg.pose.position.z

        if not self.init_pose_recorded:
            self.x0 = x_rel
            self.start_time = time.time()
            self.altitude_times, self.altitude_traj = self.generate_descent_trajectory()
            self.pid_integral = 0.0
            self.pid_prev_error = 0.0
            self.init_pose_recorded = True
            self.get_logger().info(f"[Landing Init] Starting PID guidance from x0 = {self.x0:.2f} m")
            return

        # PID Controller for airspeed
        dt = 0.5
        error = x_rel
        self.pid_integral += error * dt
        derivative = (error - self.pid_prev_error) / dt
        self.pid_prev_error = error

        kp, ki, kd = 0.5, 0.0, 0.1
        delta_v = kp * error + ki * self.pid_integral + kd * derivative
        desired_airspeed = max(self.vmin, min(self.v0, self.vmin + delta_v))

        # If x_rel > threshold, don't descend yet
        x_threshold = 12.0
        if x_rel > x_threshold:
            self.current_z = self.fixed_altitude
        else:
            t = time.time() - self.start_time
            if t >= self.altitude_times[-1]:
                self.get_logger().info("✅ Landing trajectory complete. Holding position.")
                return
            idx = min(int(t / 0.5), len(self.altitude_traj) - 1)
            self.current_z = max(0.5, self.altitude_traj[idx])

        # Send MAVLink DO_CHANGE_SPEED
        msg1 = self.vehicle.message_factory.command_long_encode(
            0, 0,
            178,
            0,
            1,
            desired_airspeed,
            -1, 0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg1)

        self.get_logger().info(
            f"[Landing] x_rel={x_rel:.2f} → airspeed={desired_airspeed:.2f} m/s | alt={self.current_z:.2f} m"
        )

    def shutdown_node(self):
        self.get_logger().info("🛑 Shutting down node")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GpsFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
