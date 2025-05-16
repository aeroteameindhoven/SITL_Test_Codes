import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from pymavlink import mavutil
from geopy.distance import geodesic
import math
import threading
import matplotlib.pyplot as plt

# --- PID Controller ---
class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# --- Quaternion Helper ---
def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ]

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_dronekit')

        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 15)
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        self.fixed_altitude = 10.0
        self.last_lat, self.last_lon = None, None
        self.uav_lat, self.uav_lon = None, None
        self.last_time = time.time()
        self.start_time = self.last_time

        self.prius_last_lat = None
        self.prius_last_lon = None


        # PID controllers
        self.airspeed_pid = PID(kp=0.08, ki=0.04, kd=0.0, integral_limit=5.0)
        self.altitude_pid = PID(kp=0.052, ki=0.0095, kd=0.25, integral_limit=5.5)
        self.heading_to_roll_pid = PID(kp=0.8, ki=0.01, kd=0.2, integral_limit=0.2)
        self.distance_pid = PID(kp=1.0, ki=0.0, kd=0.0, integral_limit=5.0)

        self.target_heading_deg = 135.0  # Set desired heading here
        self.log_roll_cmd = []
        self.log_time = []
        self.log_distance_error = []
        self.log_actual_distance = []
        self.log_altitude = []
        self.log_heading = []
        self.log_desired_heading = []
        self.run_duration = 200.0  # seconds

        self.base_throttle = 0.55

        self.takeoff(self.fixed_altitude)
        time.sleep(2)
        self.warmup_throttle()
        self.start_prius_movement()

        airspeed = self.vehicle.airspeed or 0.0
        airspeed_error = 17.0 - airspeed
        if self.airspeed_pid.ki > 0:
            self.airspeed_pid.integral = (self.base_throttle - self.airspeed_pid.kp * airspeed_error) / self.airspeed_pid.ki

        # Start threads
        self.start_attitude_thread()  # 100 Hz control loop
        self.start_gps_thread()       # 5 Hz GPS sending

    def warmup_throttle(self):
        q_init = euler_to_quaternion(0.0, 0.0, 0.0)
        for _ in range(20):
            self.vehicle._master.mav.set_attitude_target_send(
                0,
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                0b00000100,
                q_init,
                0.0, 0.0, 0.0,
                self.base_throttle
            )
            time.sleep(0.01)

    def gps_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude

        # Filtered update
        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon

        # Store previous lat/lon before updating
        self.prius_last_lat = self.last_lat
        self.prius_last_lon = self.last_lon

        self.last_lat = lat
        self.last_lon = lon

    def offset_gps(self, lat, lon, bearing_rad, distance_m):
    
        """Returns a new GPS point (lat, lon) offset by distance_m *behind* bearing_rad.
        """
        R = 6378137.0  # Earth radius in meters
        bearing = bearing_rad + math.pi  # Opposite direction

        new_lat = math.asin(
            math.sin(math.radians(lat)) * math.cos(distance_m / R) +
            math.cos(math.radians(lat)) * math.sin(distance_m / R) * math.cos(bearing)
        )
        new_lon = math.radians(lon) + math.atan2(
            math.sin(bearing) * math.sin(distance_m / R) * math.cos(math.radians(lat)),
            math.cos(distance_m / R) - math.sin(math.radians(lat)) * math.sin(new_lat)
        )

        return math.degrees(new_lat), math.degrees(new_lon)
    
    def projected_distance_along_heading(self, car_lat, car_lon, car_heading_rad, uav_lat, uav_lon):
        """
        Projects the vector from the car to the UAV onto the car's heading direction.
        Returns positive if the UAV is behind the car, negative if ahead.
        """
        # Approximate flat Earth conversion (good for short distances)
        d_north = geodesic((car_lat, car_lon), (uav_lat, car_lon)).meters
        d_east = geodesic((car_lat, car_lon), (car_lat, uav_lon)).meters
        if uav_lat < car_lat:
            d_north *= -1
        if uav_lon < car_lon:
            d_east *= -1

        heading_x = math.sin(car_heading_rad)
        heading_y = math.cos(car_heading_rad)

        dx = d_east
        dy = d_north

        projection = dx * heading_x + dy * heading_y
        return -projection
    
    def wrap_angle_deg(self, angle):
        """Wrap angle to [-180, 180] degrees."""
        return ((angle + 180) % 360) - 180

    def start_gps_thread(self):
        def send_goto_loop():
            rate = 0.2  # 5 Hz
            while rclpy.ok():
                if self.last_lat is not None:
                    target = LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude)
                    self.vehicle.simple_goto(target)
                time.sleep(rate)

        thread = threading.Thread(target=send_goto_loop)
        thread.daemon = True
        thread.start()

    def attitude_loop(self):
        log_counter = 0
        while rclpy.ok():
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            elapsed = now - self.start_time

            if self.last_lat is None or self.last_lon is None or (now - self.start_time) < 23.0:
                time.sleep(0.01)
                continue

            self.uav_lat = self.vehicle.location.global_relative_frame.lat
            self.uav_lon = self.vehicle.location.global_relative_frame.lon
            altitude = self.vehicle.location.global_relative_frame.alt or 0.0
            airspeed = self.vehicle.airspeed or 0.0

            if self.prius_last_lat is not None and self.prius_last_lon is not None:
                car_heading = self.compute_bearing(self.prius_last_lat, self.prius_last_lon,
                                                self.last_lat, self.last_lon)
                dist_to_target = self.projected_distance_along_heading(self.last_lat, self.last_lon,
                                                                car_heading,
                                                                self.uav_lat, self.uav_lon)
            else:
                # Fallback to Euclidean distance if no heading info
                dist_to_target = geodesic((self.uav_lat, self.uav_lon), (self.last_lat, self.last_lon)).meters

            distance_error = dist_to_target - 10.0

            # --- Closure rate ---
            if hasattr(self, "prev_dist"):
                closure_rate = (dist_to_target - self.prev_dist) / dt
            else:
                closure_rate = 0.0
            self.prev_dist = dist_to_target

            if elapsed > self.run_duration:
                self.get_logger().info("✅ Run complete. Shutting down...")
                rclpy.shutdown()
                break

            # --- Target airspeed with PID ---
            target_airspeed = 16.0 + self.distance_pid.update(distance_error, dt)  # baseline matching speed

            # --- Smooth slow-down near target ---
            if abs(distance_error) < 10.0 and closure_rate < -0.2:
                decel = min(2.0, 0.25 * (10.0 - abs(distance_error)))
                target_airspeed -= decel

            # --- Dynamic braking zone ---
            brake_zone = 25.0
            if dist_to_target < brake_zone and closure_rate < -0.3:
                brake_ratio = (brake_zone - dist_to_target) / brake_zone
                throttle_cut = 1.1 * brake_ratio
                target_airspeed *= max(0.0, (1.0 - throttle_cut))
                target_airspeed = max(target_airspeed, 10.0)

            # --- Hard clamp if very close and closing fast ---
            if distance_error < 2.0 and closure_rate < -0.5:
                target_airspeed = min(target_airspeed, 12.0)

            # --- Final clamp ---
            target_airspeed = max(10.0, min(target_airspeed, 22.0))

            # --- Altitude + Pitch ---
            altitude_error = self.fixed_altitude - altitude
            pitch_cmd = self.altitude_pid.update(altitude_error, dt)
            pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))

            # --- Throttle for airspeed ---
            airspeed_error = target_airspeed - airspeed
            throttle_adjust = self.airspeed_pid.update(airspeed_error, dt)
            throttle_cmd = self.base_throttle + throttle_adjust
            throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

            # --- Heading control ---
            heading_deg = self.vehicle.heading or 0.0
            heading_rad = math.radians(heading_deg)

            if self.prius_last_lat is not None and self.prius_last_lon is not None:
                prius_heading_rad = self.compute_bearing(self.prius_last_lat, self.prius_last_lon,
                                                        self.last_lat, self.last_lon)

                if self.is_uav_ahead(self.last_lat, self.last_lon, prius_heading_rad,
                                    self.uav_lat, self.uav_lon):
                    behind_lat, behind_lon = self.offset_gps(self.last_lat, self.last_lon, prius_heading_rad, 10.0)
                    self.get_logger().warn("UAV is ahead of the car — forcing retreat")
                else:
                    behind_lat, behind_lon = self.offset_gps(self.last_lat, self.last_lon, prius_heading_rad, 10.0)

                target_heading_rad = self.compute_bearing(self.uav_lat, self.uav_lon, behind_lat, behind_lon)
            else:
                target_heading_rad = self.compute_bearing(self.uav_lat, self.uav_lon, self.last_lat, self.last_lon)

            heading_error = ((target_heading_rad - heading_rad + math.pi) % (2 * math.pi)) - math.pi

            if math.isfinite(altitude) and math.isfinite(heading_deg) and math.isfinite(target_heading_rad):
                elapsed = now - self.start_time
                self.log_time.append(elapsed)
                self.log_distance_error.append(distance_error)
                self.log_actual_distance.append(dist_to_target)
                self.log_altitude.append(altitude)
                self.log_heading.append((heading_deg))
                self.log_desired_heading.append((math.degrees(target_heading_rad)))

            roll_cmd_deg = self.heading_to_roll_pid.update(heading_error, dt)
            roll_cmd_deg = max(min(roll_cmd_deg, 25), -25)

            if self.log_roll_cmd:
                roll_cmd_deg = 0.2 * roll_cmd_deg + 0.8 * self.log_roll_cmd[-1]
            self.log_roll_cmd.append(roll_cmd_deg)

            roll_cmd_rad = math.radians(roll_cmd_deg)
            if not math.isfinite(roll_cmd_rad):
                roll_cmd_rad = 0.0

            # --- Attitude Command ---
            q = euler_to_quaternion(roll_cmd_rad, pitch_cmd, 0.0)
            self.vehicle._master.mav.set_attitude_target_send(
                int((now - self.start_time) * 1000),
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                0b00000100,
                q,
                0.0, 0.0, 0.0,
                throttle_cmd
            )

            if log_counter % 10 == 0:
                self.get_logger().info(
                    f"Dist: {dist_to_target:.1f}m | TgtAS: {target_airspeed:.2f} | AS: {airspeed:.2f} | "
                    f"Alt: {altitude:.2f} | Throttle: {throttle_cmd:.2f} | Pitch(deg): {math.degrees(pitch_cmd):.2f} | "
                    f"Roll(deg): {roll_cmd_deg:.2f} | Heading: {heading_deg:.2f}"
                )
            log_counter += 1
            time.sleep(0.01)

    def compute_bearing(self, lat1, lon1, lat2, lon2):
        """
        Compute the bearing from (lat1, lon1) to (lat2, lon2) in radians.
        """
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
        bearing = math.atan2(x, y)
        return bearing  # in radians


    def start_attitude_thread(self):
        thread = threading.Thread(target=self.attitude_loop)
        thread.daemon = True
        thread.start()

    def speed_profile(self, dist, max_speed=25.0, min_speed=15.0, slowdown_radius=100.0):
        if dist > slowdown_radius:
            return max_speed
        else:
            return min_speed + (max_speed - min_speed) * (dist / slowdown_radius)

    def is_uav_ahead(self, car_lat, car_lon, car_heading_rad, uav_lat, uav_lon):
        """
        Check if UAV is in front of the car (along the heading vector).
        Returns True if UAV is ahead.
        """
        # Convert to flat Earth frame (meters), small distance approximation
        d_north = geodesic((car_lat, car_lon), (uav_lat, car_lon)).meters
        d_east = geodesic((car_lat, car_lon), (car_lat, uav_lon)).meters
        if uav_lat < car_lat:
            d_north *= -1
        if uav_lon < car_lon:
            d_east *= -1

        # Project UAV vector onto heading direction
        dx = d_east
        dy = d_north
        heading_dx = math.sin(car_heading_rad)
        heading_dy = math.cos(car_heading_rad)

        dot_product = dx * heading_dx + dy * heading_dy
        return dot_product > 0  # UAV is in front if projection is positive


    def takeoff(self, target_altitude):
        self.get_logger().info("Arming UAV...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            self.get_logger().info("Waiting for UAV to arm...")
            time.sleep(1)

        self.get_logger().info(f"Taking off to {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            self.get_logger().info(f"Altitude: {altitude:.1f}m")
            if altitude >= target_altitude * 0.95:
                self.get_logger().info("Reached target altitude!")
                break
            time.sleep(1)

    def start_prius_movement(self):
        vel_msg = Twist()
        vel_msg.linear.x = 16.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("Prius moving forward at 16 m/s...")

def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()
    try:
        rclpy.spin(gps_follower)
    except KeyboardInterrupt:
        pass
    finally:
        gps_follower.destroy_node()

        # --- Plot after node is destroyed ---
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 5))
        plt.plot(gps_follower.log_time, gps_follower.log_actual_distance, label="UAV Distance to Car (m)")
        plt.axhline(10.0, linestyle='--', color='gray', label="Target Distance (10m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Distance (m)")
        plt.title("UAV Following Distance Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        plt.figure(figsize=(10, 5))
        plt.plot(gps_follower.log_time, gps_follower.log_altitude, label="UAV Altitude (m)")
        plt.axhline(10.0, linestyle='--', color='gray', label="Target Altitude")
        plt.xlabel("Time (s)")
        plt.ylabel("Distance (m)")
        plt.title("UAV Altitude Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        plt.figure(figsize=(10, 5))
        plt.plot(gps_follower.log_time, gps_follower.log_heading, label="Actual Heading (deg)")
        plt.plot(gps_follower.log_time, gps_follower.log_desired_heading, label="Desired Heading (deg)", linestyle='--')
        plt.xlabel("Time (s)")
        plt.ylabel("Heading (degrees)")
        plt.title("Actual vs Desired UAV Heading")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    main()