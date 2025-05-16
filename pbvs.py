import rclpy
from rclpy.node import Node
import numpy as np
import time
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped

# Connect to MAVLink (adjust for real UAV or SITL)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# PID Controllers for Fixed-Wing Control
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        """Calculate PID output"""
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

# PID Controllers
pid_roll = PIDController(Kp=0.6, Ki=0.1, Kd=0.2)  # Lateral correction
pid_pitch = PIDController(Kp=0.8, Ki=0.1, Kd=0.2)  # Altitude control
pid_thrust = PIDController(Kp=0.5, Ki=0.05, Kd=0.1)  # Airspeed control

# Desired Landing Parameters
TARGET_AIRSPEED = 16.0  # m/s
TARGET_ALTITUDE = -1.0  # Target altitude relative to landing surface
TARGET_LATERAL_POSITION = 0.0  # Ideal lateral x-position

class PBVSLandingNode(Node):
    def __init__(self):
        super().__init__('pbvs_landing')

        # Subscribe to AprilTag Pose Topic in ROS 2
        self.create_subscription(PoseStamped, '/apriltag/pose', self.apriltag_pose_callback, 10)

        # Initialize Pose and Airspeed
        self.april_tag_pose = None
        self.airspeed = TARGET_AIRSPEED  # Default airspeed

        # Timer for main control loop
        self.create_timer(0.1, self.control_loop)  # 10Hz loop

    def apriltag_pose_callback(self, msg):
        """ROS 2 Callback for AprilTag Pose"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.april_tag_pose = (x, y, z)  # No yaw

    def get_airspeed(self):
        """Retrieve airspeed from ArduPilot (VFR_HUD message)"""
        msg = master.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            self.airspeed = msg.airspeed
        return self.airspeed

    def compute_target_controls(self):
        """Compute roll, pitch, and thrust based on AprilTag pose and airspeed"""
        if self.april_tag_pose is None:
            return None

        x, y, z = self.april_tag_pose  # Pose estimation data

        # Compute Errors
        error_altitude = TARGET_ALTITUDE - z
        error_lateral = TARGET_LATERAL_POSITION - x
        error_airspeed = TARGET_AIRSPEED - self.get_airspeed()

        # PID-based Corrections
        roll = np.clip(pid_roll.compute(error_lateral), -np.radians(25), np.radians(25))  # Max ±25° bank angle
        pitch = np.clip(pid_pitch.compute(error_altitude), -np.radians(15), np.radians(15))  # Max ±15° pitch
        thrust = np.clip(0.5 + pid_thrust.compute(error_airspeed), 0.3, 1.0)  # Adjust thrust (30%-100%)

        return roll, pitch, thrust

    def send_fixed_wing_control(self, roll, pitch, thrust):
        """Send MAVLink SET_ATTITUDE_TARGET command for fixed-wing UAV"""
        q = self.euler_to_quaternion(roll, pitch, 0)  # No yaw control
        master.mav.set_attitude_target_send(
            int(time.time() * 1e6),
            master.target_system,
            master.target_component,
            0b00000000,  # Use absolute roll, pitch
            q,  # Quaternion
            0, 0, 0,  # No angular rate control
            thrust
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]

    def control_loop(self):
        """Main Control Loop"""
        controls = self.compute_target_controls()
        if controls:
            roll, pitch, thrust = controls
            self.send_fixed_wing_control(roll, pitch, thrust)
            self.get_logger().info(f"Sent Controls -> Roll: {np.degrees(roll):.2f}, Pitch: {np.degrees(pitch):.2f}, Thrust: {thrust:.2f}")

# Main entry point
def main(args=None):
    rclpy.init(args=args)
    node = PBVSLandingNode()
    rclpy.spin(node)  # Keep ROS 2 node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
