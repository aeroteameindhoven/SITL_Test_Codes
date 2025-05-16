import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil

class UAVLandingController(Node):
    def __init__(self):
        super().__init__('uav_landing_controller')

        # ✅ Connect to UAV with timeout
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True, timeout=60)
        self.get_logger().info("Connected to UAV!")

        # ✅ MAVLink setup
        self.mavlink_conn = mavutil.mavlink_connection("udp:127.0.0.1:14550")

        # ✅ ROS 2 Subscribers
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)
        self.create_subscription(PoseStamped, '/apriltag/pose', self.apriltag_callback, 10)

        # ✅ Parameters
        self.fixed_altitude = 10.0
        self.speed_matching_duration = 23.0
        self.start_time = time.time()

        # ✅ State tracking
        self.following_started = False
        self.using_landing_controller = False
        self.last_lat, self.last_lon = None, None  # Store last GPS fix

        # ✅ Takeoff & Start Movement
        self.takeoff(self.fixed_altitude)
        time.sleep(2)

        # ✅ Start control loop (runs every 0.1 sec)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        """Main loop: GPS speed match for 23s, then switch to landing control."""
        elapsed_time = time.time() - self.start_time

        if elapsed_time < self.speed_matching_duration:
            self.following_started = True
        else:
            if not self.using_landing_controller:
                self.get_logger().info("Switching to landing controller!")
                self.using_landing_controller = True

        if self.using_landing_controller:
            self.run_landing_controller()
        else:
            self.run_gps_follower()

    def run_gps_follower(self):
        """Follows UGV using GPS speed matching (simple_goto)."""
        if self.last_lat is None or self.last_lon is None:
            self.get_logger().warn("GPS follower: No valid GPS fix yet!")
            return

        target_location = LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude)
        self.get_logger().info(f"Following GPS target: {self.last_lat}, {self.last_lon}")
        self.vehicle.simple_goto(target_location)

    def run_landing_controller(self):
        """Uses AprilTag for precision landing."""
        if self.x_uav is None:
            self.get_logger().warn("Landing controller: No AprilTag data yet!")
            return

        V_cmd = self.compute_airspeed_command()
        self.send_do_change_speed(V_cmd)

    def apriltag_callback(self, msg):
        """Updates UAV pose from AprilTag."""
        self.x_uav = -msg.pose.position.x
        self.h_uav = msg.pose.position.z

    def gps_callback(self, msg):
        """Updates GPS position while running both controllers."""
        lat, lon = msg.latitude, msg.longitude
        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon
        self.last_lat, self.last_lon = lat, lon

    def compute_airspeed_command(self):
        """Computes speed based on relative position."""
        position_error = -self.x_uav
        V_cmd = 21.0 + 0.05 * position_error
        return max(V_cmd, 16.0)

    def send_do_change_speed(self, V_cmd):
        """Sends MAVLink DO_CHANGE_SPEED without blocking."""
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # Target system, target component
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # Command ID
            0,  # Confirmation
            0, V_cmd, -1, 0, 0, 0, 0  # Params
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()  # Ensure message is sent
        self.get_logger().info(f"Set UAV speed to {V_cmd:.2f} m/s")

    def takeoff(self, target_altitude):
        """Takes off the UAV to the given altitude with timeout."""
        self.get_logger().info("Arming UAV...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        timeout = time.time() + 15  # Max 15 sec to arm
        while not self.vehicle.armed:
            self.get_logger().info("Waiting for UAV to arm...")
            time.sleep(1)
            if time.time() > timeout:
                self.get_logger().error("UAV failed to arm!")
                return

        self.get_logger().info(f"Taking off to {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)

        timeout = time.time() + 30  # Max 30 sec to reach altitude
        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            if altitude >= target_altitude * 0.95:
                self.get_logger().info("Reached target altitude!")
                break
            if time.time() > timeout:
                self.get_logger().error("Takeoff timed out!")
                break
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = UAVLandingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
