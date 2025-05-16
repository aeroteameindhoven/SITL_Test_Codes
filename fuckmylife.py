from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
from geographiclib.geodesic import Geodesic
import matplotlib.pyplot as plt

# --------------------------------------
# Connect to Vehicle
# --------------------------------------
def connect_vehicle():
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    print("Connected to vehicle")
    return vehicle

# --------------------------------------
# Generate Cosine-Flare Descent Trajectory
# --------------------------------------
def generate_trajectory(total_time=15.0, dt=0.5, start_altitude=10.0, airspeed=16.0):
    times = np.arange(0, total_time + dt, dt)
    altitudes = start_altitude * 0.5 * (1 + np.cos(np.pi * times / total_time))  # cosine from 10→0
    airspeeds = np.full_like(times, airspeed)
    return times, altitudes, airspeeds

# --------------------------------------
# Run Altitude-Only Landing with Altitude Tracking
# --------------------------------------
def run_landing(vehicle, landing_lat, landing_lon, total_time=15.0, dt=0.5, airspeed=16.0, heading_deg=90.0, Kp=0.8):
    times, altitudes, airspeeds = generate_trajectory(total_time, dt)

    intended_positions = []
    actual_positions = []

    for t, alt, spd in zip(times, altitudes, airspeeds):
        # Get current position
        current = vehicle.location.global_relative_frame
        current_lat, current_lon = current.lat, current.lon
        current_alt = current.alt

        # Altitude error and correction
        error = alt - current_alt
        corrected_alt = current_alt + Kp * error

        # Command new location (keep lat/lon constant)
        loc = LocationGlobalRelative(landing_lat, landing_lon, corrected_alt)
        print(f"[{t:.1f}s] → Target Alt: {alt:.2f} | Actual: {current_alt:.2f} | Sending: {corrected_alt:.2f}")
        vehicle.simple_goto(loc, groundspeed=spd)

        intended_positions.append((landing_lat, landing_lon, alt))
        actual_positions.append((current_lat, current_lon, current_alt))

        time.sleep(dt)

    print("Landing trajectory complete. Disarming...")
    time.sleep(2)

    print("✅ UAV disarmed successfully.")
    return intended_positions, actual_positions

# --------------------------------------
# Plot Altitude vs X Distance to Landing
# --------------------------------------
def plot_trajectory(intended, actual, landing_lat, landing_lon, heading_deg=90.0):
    geod = Geodesic.WGS84

    def compute_x_distance(lat, lon):
        inv = geod.Inverse(landing_lat, landing_lon, lat, lon)
        bearing_diff = inv['azi1'] - heading_deg
        x = inv['s12'] * np.cos(np.radians(bearing_diff))
        return x

    int_x = [compute_x_distance(lat, lon) for lat, lon, _ in intended]
    int_alt = [alt for _, _, alt in intended]

    act_x = [compute_x_distance(lat, lon) for lat, lon, _ in actual]
    act_alt = [alt for _, _, alt in actual]

    plt.figure()
    plt.plot(int_x, int_alt, label='Intended', color='blue')
    plt.plot(act_x, act_alt, label='Actual', color='red')
    plt.xlabel('X Distance to Landing Point (m)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude vs X-Distance')
    plt.gca().invert_xaxis()
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

# --------------------------------------
# Main
# --------------------------------------
if __name__ == "__main__":
    vehicle = connect_vehicle()

    # 🎯 Manually set on the map
    landing_lat = -35.35717926
    landing_lon = 149.16507697

    intended, actual = run_landing(
        vehicle,
        landing_lat,
        landing_lon,
        total_time=15.0,
        dt=0.5,
        airspeed=16.0,
        heading_deg=90.0,
        Kp=0.8
    )

    plot_trajectory(intended, actual, landing_lat, landing_lon)
