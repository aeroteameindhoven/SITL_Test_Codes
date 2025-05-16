import time
import numpy as np
from pymavlink import mavutil
import scipy.interpolate as interp

class UAVController:
    def __init__(self, connection_string):
        """Initialize MAVLink connection and verify communication."""
        print("🔄 Connecting to MAVLink SITL...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print(f"✅ MAVLink connection established with system {self.master.target_system}, component {self.master.target_component}")

    def min_jerk_trajectory(self, wpts, tpts, tsamples):
        """ Compute minimum jerk trajectory. """
        q, qd, qdd, qddd = [], [], [], []

        for i in range(wpts.shape[0]):
            cs = interp.CubicSpline(tpts, wpts[i], bc_type=((1, 0), (1, 0)))
            q.append(cs(tsamples))      # Position
            qd.append(cs(tsamples, 1))  # Velocity
            qdd.append(cs(tsamples, 2)) # Acceleration
            qddd.append(cs(tsamples, 3))# Jerk
        
        return np.array(q), np.array(qd), np.array(qdd), np.array(qddd)

    def compute_attitude(self, a_y, a_z, g=9.81):
        """ Compute roll and pitch based on acceleration. """
        roll = np.arctan2(a_y, g)
        alpha = np.radians(5.0)  # Trim AoA
        pitch = alpha + np.arcsin(np.clip(a_z / g))
        return roll, pitch

    def euler_to_quaternion(self, roll, pitch, yaw=0):
        """ Convert roll, pitch, yaw to quaternion. """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ]
        return q

    def send_attitude_target(self, roll, pitch, yaw=0):
        """ Send roll and pitch to the UAV using ATTITUDE_TARGET. """
        q = self.euler_to_quaternion(roll, pitch, yaw)

        print(f"🚀 ATTITUDE_TARGET | Roll={np.degrees(roll):.2f}°, "
              f"Pitch={np.degrees(pitch):.2f}°, Yaw={np.degrees(yaw):.2f}°, Quaternion={q}")

        self.master.mav.set_attitude_target_send(
            int(self.master.time_since('BOOT') * 1e3),
            self.master.target_system,
            self.master.target_component,
            0b00000000,  # Type mask: ignore body rates
            q,           # Quaternion
            0, 0, 0,     # Body rates (ignored)
            0            # Thrust = 0
        )

    def send_velocity_change(self, vx):
        """ Send a MAVLink DO_CHANGE_SPEED command for vx """
        print(f"📤 DO_CHANGE_SPEED | Sending vx = {vx:.2f} m/s")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,  # Confirmation
            1,      # Airspeed change
            vx,     # Speed in m/s
            -1,     # Throttle (ignored)
            0, 0, 0, 0
        )

    def execute_trajectory(self, wpts, tpts, tsamples):
        """ Compute trajectory and send attitude + vx commands """
        print("📡 Computing trajectory...")
        q, qd, qdd, _ = self.min_jerk_trajectory(wpts, tpts, tsamples)
        
        start_time = time.time()

        for i in range(len(tsamples)):
            ax, ay, az = qdd[:, i]
            vx = qd[0, i]
            roll, pitch = self.compute_attitude(ax, ay, az)

            print(f"⏳ t={tsamples[i]:.2f}s | Roll={np.degrees(roll):.2f}°, "
                  f"Pitch={np.degrees(pitch):.2f}°, vx={vx:.2f} m/s")

            self.send_attitude_target(roll, pitch)
            self.send_velocity_change(vx)

            next_time = start_time + tsamples[i]
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("🎯 Trajectory execution complete.")

# =======================
# 🚀 Run the SITL Test
# =======================

if __name__ == '__main__':
    controller = UAVController('udp:127.0.0.1:14550')  # Change if needed

    wpts = np.array([
        [-10,  5,  20],  # X
        [-1,   3,   8],  # Y
        [10,   8,   5]   # Z
    ])
    tpts = np.array([0, 5, 10])  # Time points
    tsamples = np.linspace(0, 10, 50)

    controller.execute_trajectory(wpts, tpts, tsamples)
