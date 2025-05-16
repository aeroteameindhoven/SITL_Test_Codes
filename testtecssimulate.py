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
import csv

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_single_loop')

        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 15)
        #self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        self.fixed_altitude = 20.0
        self.last_lat, self.last_lon = None, None
        self.uav_lat, self.uav_lon = None, None
        self.start_time = time.time()
        self.control_start_time = None
        self.run_duration = 400.0
        self.last_time = time.time()
        self.pid_integral = 0.0
        self.prev_error = 0.0
        self.last_target_airspeed = 14.0  # For rate-limiting
        self.prev_error = 0.0            # For derivative control
        self.smoothed_target_airspeed = None
        self.trajectory_start_time = None
        self.trajectory_duration = 10.0  # seconds over which to reach the new goal
        self.initial_ref_distance = None
        self.final_ref_distance = None
        self.initial_altitude = None
        self.final_altitude = None

        # Logging
        self.log_time = []
        self.log_distance_error = []
        self.log_chirp_input = []
        self.log_altitude = []
        self.log_ref_distance = []
        self.log_actual_airspeed = []
        self.log_actual_distance = []
        self.log_horizontal_distance = []
        self.log_actual_lat = []
        self.log_actual_lon = []
        self.log_target_lat = []
        self.log_target_lon = []

        self.takeoff(self.fixed_altitude)
        time.sleep(2)

        # Set UAV position here to use in target initialization
        loc = self.vehicle.location.global_relative_frame
        self.uav_lat = loc.lat
        self.uav_lon = loc.lon

        self.initialize_moving_target()
        self.start_main_loop()

    def gps_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude
        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon
        self.last_lat = lat
        self.last_lon = lon

    def initialize_moving_target(self):
        # Use UAV's current heading and position to set initial car state
        heading_deg = self.vehicle.heading
        self.sim_start_time = time.time()
        self.sim_start_lat = self.uav_lat
        self.sim_start_lon = self.uav_lon
        self.sim_heading_rad = math.radians(heading_deg)

        # Offset initial target 40 meters ahead
        earth_radius = 6378137.0
        d_lat = (40.0 * math.cos(self.sim_heading_rad)) / earth_radius
        d_lon = (40.0 * math.sin(self.sim_heading_rad)) / (earth_radius * math.cos(math.radians(self.sim_start_lat)))

        self.sim_start_lat += math.degrees(d_lat)
        self.sim_start_lon += math.degrees(d_lon)

    def compute_moving_target(self):
        t = time.time() - self.sim_start_time
        speed = 16.0  # m/s
        dist = speed * t
        earth_radius = 6378137.0

        d_lat = (dist * math.cos(self.sim_heading_rad)) / earth_radius
        d_lon = (dist * math.sin(self.sim_heading_rad)) / (earth_radius * math.cos(math.radians(self.sim_start_lat)))

        new_lat = self.sim_start_lat + math.degrees(d_lat)
        new_lon = self.sim_start_lon + math.degrees(d_lon)
        return new_lat, new_lon

    def reference_distance(self, t):
        if self.trajectory_start_time is None or t < self.trajectory_start_time:
            return self.initial_ref_distance  # constant before trajectory
        elif t >= self.trajectory_start_time + self.trajectory_duration:
            return self.final_ref_distance
        else:
            # Linear interpolation
            ratio = (t - self.trajectory_start_time) / self.trajectory_duration
            return self.initial_ref_distance + ratio * (self.final_ref_distance - self.initial_ref_distance)

    def reference_altitude(self, t):
        if self.trajectory_start_time is None or t < self.trajectory_start_time:
            return self.initial_altitude
        elif t >= self.trajectory_start_time + self.trajectory_duration:
            return self.final_altitude
        else:
            ratio = (t - self.trajectory_start_time) / self.trajectory_duration
            return self.initial_altitude + ratio * (self.final_altitude - self.initial_altitude)

    def maybe_start_trajectory(self, elapsed_control_time, current_altitude, current_distance):
        if self.trajectory_start_time is None:
            self.trajectory_start_time = elapsed_control_time
            self.initial_ref_distance = current_distance
            self.final_ref_distance = self.initial_ref_distance - 5.0
            self.initial_altitude = current_altitude
            self.final_altitude = self.initial_altitude - 5.0
            self.get_logger().info(
                f"📉 Starting trajectory: Alt {self.initial_altitude:.1f} -> {self.final_altitude:.1f}, "
                f"Dist {self.initial_ref_distance:.1f} -> {self.final_ref_distance:.1f}"
            )

    def compute_airspeed_from_error(self, distance, t):
        ref_dist = self.reference_distance(t)
        error = distance - ref_dist  # ✅ Signed error to handle overshoot

        dt = max(0.001, time.time() - self.last_time)

        # Updated PID (more damped, less aggressive derivative)
        k_p = 1
        k_i = 0
        k_d = 0
        
        # Integral term
        self.pid_integral += error * dt

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        correction = k_p * error + k_i * self.pid_integral + k_d * derivative

        base_speed = 16.0
        raw_target_airspeed = base_speed + correction
        raw_target_airspeed = max(14.0, min(20.0, raw_target_airspeed))

        self.last_time = time.time()
        return raw_target_airspeed, ref_dist, error

    def start_main_loop(self):
        def loop():
            while rclpy.ok():
                now = time.time()
                elapsed_total = now - self.start_time

                # Get UAV state FIRST
                uav_loc = self.vehicle.location.global_relative_frame
                self.uav_lat = uav_loc.lat
                self.uav_lon = uav_loc.lon
                altitude = uav_loc.alt or 0.0
                airspeed = self.vehicle.airspeed or 0.0

                # Now compute simulated target
                dt = now - self.last_time
                sim_lat, sim_lon = self.compute_moving_target()
                if sim_lat is None or sim_lon is None:
                    time.sleep(1.0 / 5.0)
                    continue

                if self.trajectory_start_time is not None:
                    target_alt = self.reference_altitude(now - self.start_time)
                else:
                    target_alt = altitude  # Stay level until trajectory starts

                target = LocationGlobalRelative(sim_lat, sim_lon, target_alt)
                self.vehicle.simple_goto(target)
                self.last_lat = sim_lat
                self.last_lon = sim_lon

                dist_to_target = geodesic((self.uav_lat, self.uav_lon), (self.last_lat, self.last_lon)).meters

                # Start control after 23s
                if self.control_start_time is None and elapsed_total >= 23.0:
                    self.control_start_time = now
                    elapsed = 0.0  # define it before using
                    self.maybe_start_trajectory(elapsed, altitude, dist_to_target)

                if self.control_start_time is not None:
                    elapsed = now - self.control_start_time
                    target_airspeed, ref_dist, error = self.compute_airspeed_from_error(dist_to_target, elapsed)

                    self.vehicle._master.mav.command_long_send(
                        self.vehicle._master.target_system,
                        self.vehicle._master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                        0,
                        0,
                        target_airspeed,
                        -1, 0, 0, 0, 0
                    )

                    self.log_time.append(elapsed)
                    self.log_distance_error.append(error)
                    self.log_chirp_input.append(target_airspeed)
                    self.log_altitude.append(altitude)
                    self.log_ref_distance.append(ref_dist)
                    self.log_actual_lat.append(self.uav_lat)
                    self.log_actual_lon.append(self.uav_lon)
                    self.log_target_lat.append(self.last_lat)
                    self.log_target_lon.append(self.last_lon)


                    if self.log_actual_airspeed:
                        smoothed_airspeed = 0.8 * self.log_actual_airspeed[-1] + 0.2 * airspeed
                    else:
                        smoothed_airspeed = airspeed

                    self.log_actual_airspeed.append(smoothed_airspeed)
                    self.log_actual_distance.append(dist_to_target)

                    # Diagnostic log for what phase we're in
                    if self.trajectory_start_time is not None:
                        if elapsed < self.trajectory_start_time:
                            traj_status = "🟡 Pre-trajectory"
                        elif elapsed < self.trajectory_start_time + self.trajectory_duration:
                            traj_status = "🟠 Trajectory active"
                        else:
                            traj_status = "🟢 Trajectory complete"

                    self.get_logger().info(
                        f"[{elapsed:.1f}s] {traj_status} | Dist: {dist_to_target:.1f} | Ref: {ref_dist:.1f} | "
                        f"Target AS: {target_airspeed:.1f} | AS: {airspeed:.1f} | Alt: {altitude:.1f} | Target: {ref_dist:.1f}m | Target Alt: {self.reference_altitude(elapsed):.1f}m"
                    )

                    if elapsed > self.run_duration:
                        self.get_logger().info("✅ Run complete. Shutting down...")
                        rclpy.shutdown()
                        break
                else:

                    self.get_logger().info(f"[{elapsed_total:.1f}s] GOTO only")

                self.last_time = now
                time.sleep(1.0 / 5.0)

        thread = threading.Thread(target=loop)
        thread.daemon = True
        thread.start()

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

def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()
    try:
        rclpy.spin(gps_follower)
    except KeyboardInterrupt:
        pass
    finally:
        gps_follower.destroy_node()

        # Save data to CSV
        with open("distance_log2.csv", "w", newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time (s)", "Target Distance (m)", "Distance Error (m)", "Altitude (m)", "Airspeed Command (m/s)", "Actual Airspeed (m/s), Horisontal Distance (m)"])
            for t, ref_d, d, alt, a_cmd, a_act, d_actual in zip(
                gps_follower.log_time,
                gps_follower.log_ref_distance,
                gps_follower.log_distance_error,
                gps_follower.log_altitude,
                gps_follower.log_chirp_input,
                gps_follower.log_actual_airspeed,
                gps_follower.log_actual_distance
            ):
                writer.writerow([t, ref_d, d, alt, a_cmd, a_act, d_actual])

        # Plot
        plt.figure(figsize=(12, 6))

        # Altitude
        plt.subplot(3, 1, 1)
        plt.plot(gps_follower.log_time, gps_follower.log_altitude, label="Altitude (m)")
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude")
        plt.title("Altitude Tracking")
        plt.grid(True)
        plt.legend()

        # Distance
        plt.subplot(3, 1, 2)
        plt.plot(gps_follower.log_time, gps_follower.log_actual_distance, label="Actual Distance", color='orange')
        plt.plot(gps_follower.log_time, gps_follower.log_ref_distance, label="Target Distance", linestyle='--', color='blue')
        plt.xlabel("Time (s)")
        plt.ylabel("Distance (m)")
        plt.title("Distance Tracking")
        plt.grid(True)
        plt.legend()

        # Airspeed
        plt.subplot(3, 1, 3)
        plt.plot(gps_follower.log_time, gps_follower.log_actual_airspeed, label="Actual Airspeed", color='orange')
        plt.plot(gps_follower.log_time, gps_follower.log_chirp_input, label="Target Airspeed", linestyle='--', color='blue')
        plt.xlabel("Time (s)")
        plt.ylabel("Airspeed (m/s)")
        plt.title("Airspeed Tracking")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

        # Trajectory plot (Actual vs. Target GPS positions)
        plt.figure(figsize=(8, 6))

        # Compute relative horizontal distances
        relative_distances = []
        for lat, lon in zip(gps_follower.log_target_lat, gps_follower.log_target_lon):
            uav_pos = (gps_follower.log_actual_lat[0], gps_follower.log_actual_lon[0])
            target_pos = (lat, lon)
            dist = geodesic(uav_pos, target_pos).meters
            relative_distances.append(dist)

        plt.plot(relative_distances, gps_follower.log_altitude, label="UAV Trajectory", color='blue')

        # Optional: Add reference line for initial to final altitude over expected range
        if gps_follower.initial_ref_distance and gps_follower.final_ref_distance:
            target_distances = [gps_follower.initial_ref_distance, gps_follower.final_ref_distance]
            target_altitudes = [gps_follower.initial_altitude, gps_follower.final_altitude]
            plt.plot(target_distances, target_altitudes, '--', label="Desired Trajectory", color='red')

        plt.xlabel("Relative Horizontal Distance (m)")
        plt.ylabel("Altitude (m)")
        plt.title("UAV Vertical Trajectory")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
