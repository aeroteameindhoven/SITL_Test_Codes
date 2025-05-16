from dronekit import connect
from pymavlink import mavutil
import time
import math
import itertools
import numpy as np
import matplotlib.pyplot as plt

# --- PID Class ---
class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.last_error) / dt if dt > 0 else 0
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

# --- Flight Test With Given PID ---
def fly_with_pid(vehicle, kp, ki, kd, duration=30.0, target_alt=48.0):
    pid = PID(kp, ki, kd, integral_limit=5)
    q_zero = euler_to_quaternion(0.0, 0.0, 0.0)
    log_time, log_alt = [], []

    print(f"→ Flying with kp={kp:.3f}, ki={ki:.3f}, kd={kd:.3f}")
    start_time = time.time()
    last_time = start_time

    while True:
        now = time.time()
        dt = now - last_time
        elapsed = now - start_time
        last_time = now
        if elapsed > duration:
            break

        current_alt = vehicle.location.global_relative_frame.alt or 0.0
        error = target_alt - current_alt
        pitch_cmd = pid.update(error, dt)
        pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-15))
        q = euler_to_quaternion(0.0, pitch_cmd, 0.0)

        vehicle._master.mav.set_attitude_target_send(
            int(elapsed * 1000),
            vehicle._master.target_system,
            vehicle._master.target_component,
            0b00000100,
            q,
            0.0, 0.0, 0.0,
            0.5  # constant throttle for test
        )

        log_time.append(elapsed)
        log_alt.append(current_alt)
        time.sleep(0.01)

    return log_time, log_alt

# --- Grid Search Autotuner ---
def grid_search_altitude_pid(vehicle):
    kp_vals = [0.025, 0.035, 0.045]
    ki_vals = [0.005, 0.01, 0.015]
    kd_vals = [0.1, 0.15, 0.2]

    best_score = float('inf')
    best_pid = None
    best_log = None

    for kp, ki, kd in itertools.product(kp_vals, ki_vals, kd_vals):
        t, alt = fly_with_pid(vehicle, kp, ki, kd)
        target = np.full_like(alt, 48.0)
        error = target - np.array(alt)
        steady_error = np.abs(np.mean(error[-20:]))
        overshoot = np.max(alt) - 48.0
        rise_idx = np.argmax(np.array(alt) > 47.9)
        rise_time = t[rise_idx] if rise_idx < len(t) else len(t)
        cost = 3*steady_error + 2*abs(overshoot) + 1.0*rise_time

        print(f"   → Score = {cost:.3f}")
        if cost < best_score:
            best_score = cost
            best_pid = (kp, ki, kd)
            best_log = (t, alt)

    print(f"\n✅ Best PID: kp={best_pid[0]:.3f}, ki={best_pid[1]:.3f}, kd={best_pid[2]:.3f} | Score={best_score:.3f}")

    # Plot the best response
    plt.figure(figsize=(8, 4))
    plt.plot(best_log[0], best_log[1], label="Altitude")
    plt.axhline(48.0, linestyle='--', color='gray', label="Target")
    plt.title("Best Altitude Response")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

# --- Main Entry Point ---
if __name__ == "__main__":
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    print("Connected!\n")

    grid_search_altitude_pid(vehicle)
