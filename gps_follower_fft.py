import time
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from dronekit import connect, VehicleMode, LocationGlobalRelative
from threading import Thread, Lock
from scipy.fft import fft, fftfreq

class PrbsDistanceLogger(Node):
    def __init__(self):
        super().__init__('prbs_distance_logger')

        # === Connect to UAV
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        # === ROS
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(NavSatFix, '/prius/gps', self.car_gps_callback, 10)

        self.car_lat = None
        self.car_lon = None
        self.last_lat = None
        self.last_lon = None
        self.fixed_altitude = 10.0
        self.last_gps_time = 0
        self.gps_lock = Lock()

        # === Load PRBS
        self.prbs_data = np.loadtxt('1to100PRBS0.0001TS.csv', delimiter=',')
        self.prbs_time = self.prbs_data[:, 0]
        self.prbs_throttle = self.prbs_data[:, 1]
        self.log_data = []
        self.prbs_started = False

        # === Takeoff and start car
        self.takeoff(self.fixed_altitude)
        time.sleep(2)
        self.start_prius_movement()
        self.start_time = time.time()

        # === Start PRBS in a real thread after 10s
        Thread(target=self.delayed_prbs_start, daemon=True).start()

    def car_gps_callback(self, msg):
        now = time.time()
        if now - self.last_gps_time < 0.5:
            return  # Limit to 2 Hz

        with self.gps_lock:
            self.car_lat = msg.latitude
            self.car_lon = msg.longitude

        self.last_gps_time = now
        self.send_gps_target(self.car_lat, self.car_lon)

    def send_gps_target(self, lat, lon):
        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon
        self.last_lat, self.last_lon = lat, lon

        if self.vehicle.mode != VehicleMode("GUIDED"):
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)

        target = LocationGlobalRelative(lat, lon, self.fixed_altitude)
        self.vehicle.simple_goto(target)
        time.sleep(0.05)

    def delayed_prbs_start(self):
        time.sleep(10)
        self.prbs_started = True
        self.get_logger().info("🔥 Starting PRBS thread with real timestamp locking")
        self.run_prbs_sequence()

    def run_prbs_sequence(self):
        master = self.vehicle._master
        start_time = time.time()

        def send_throttle(throttle, time_boot_ms):
            master.mav.set_attitude_target_send(
                time_boot_ms,
                master.target_system,
                master.target_component,
                0b00000111,
                [0, 0, 0, 0],
                0, 0, 0,
                throttle
            )

        for i in range(len(self.prbs_time)):
            desired_time = self.prbs_time[i]
            throttle = self.prbs_throttle[i]
            target_wall_time = start_time + desired_time
            delay = target_wall_time - time.time()
            if delay > 0:
                time.sleep(delay)

            now = time.time()
            t = now - start_time
            time_boot_ms = int(t * 1000)
            send_throttle(throttle, time_boot_ms)

            # Distance to car
            with self.gps_lock:
                car_lat = self.car_lat
                car_lon = self.car_lon

            if car_lat is not None and self.vehicle.location.global_frame.lat is not None:
                uav_lat = self.vehicle.location.global_frame.lat
                uav_lon = self.vehicle.location.global_frame.lon
                distance = self.compute_distance(uav_lat, uav_lon, car_lat, car_lon)
            else:
                distance = float('nan')

            self.log_data.append([t, throttle, distance])

        self.get_logger().info("✅ PRBS complete. Saving log...")
        self.save_csv_and_fft()

    def compute_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def save_csv_and_fft(self):
        arr = np.array(self.log_data)
        np.savetxt('throttle_vs_distance_log.csv', arr,
                   delimiter=',', header='time,throttle,distance', comments='')

        t = arr[:, 0]
        u = arr[:, 1]
        y = arr[:, 2]
        dt = np.mean(np.diff(t))
        f = fftfreq(len(t), dt)
        U = fft(u)
        Y = fft(y)

        print("\n=== FFT ===")
        print(f"{'Freq (Hz)':>10} | {'|U(f)|':>10} | {'|Y(f)|':>10}")
        for i in range(1, len(f)//2):
            print(f"{f[i]:10.2f} | {abs(U[i]):10.4f} | {abs(Y[i]):10.4f}")

        print("✅ Saved to 'throttle_vs_distance_log.csv'")

    def takeoff(self, target_altitude):
        self.get_logger().info("Taking off...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)

        self.vehicle.simple_takeoff(target_altitude)
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                self.get_logger().info("Altitude reached.")
                break
            time.sleep(1)

    def start_prius_movement(self):
        msg = Twist()
        msg.linear.x = 16.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("🚗 Prius moving at 16 m/s")

def main(args=None):
    rclpy.init(args=args)
    node = PrbsDistanceLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
