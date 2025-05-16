import numpy as np
import time
import matplotlib.pyplot as plt
from dronekit import connect
from scipy.signal import chirp

# === Step 1: Generate Chirp Signal ===
duration = 30.0  # seconds
sampling_rate = 100  # Hz — must be high to support 100Hz
t = np.linspace(0, duration, int(duration * sampling_rate))
f0 = 1.0   # Start frequency (Hz)
f1 = 50.0  # End frequency (Hz)
amplitude = 0.5  # Max throttle variation
offset = 0.5  # Base throttle (to stay positive)

throttle_values = offset + amplitude * chirp(t, f0=f0, f1=f1, t1=duration, method='linear')
throttle_values = np.clip(throttle_values, 0.0, 1.0)
time_stamps = t

# === Step 2: Connect to SITL using DroneKit ===
print("Connecting to SITL using DroneKit...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
master = vehicle._master
print("Connected to vehicle.")

# === Step 3: Define MAVLink throttle-only SET_ATTITUDE_TARGET ===
def set_attitude_throttle(throttle, time_boot_ms):
    master.mav.set_attitude_target_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        0b00000111,  # Ignore orientation
        [0, 0, 0, 0],
        0, 0, 0,
        throttle
    )

# === Step 4: Send Chirp Signal and Record Airspeed ===
airspeed_log = []
start_time = time.time()
print("Sending chirp throttle signal...")

for i in range(len(time_stamps)):
    target_time = time_stamps[i]
    throttle_cmd = throttle_values[i]

    # Wait for real time
    while (time.time() - start_time) < target_time:
        time.sleep(0.00001)

    current_time = time.time() - start_time
    time_boot_ms = int(current_time * 1000)

    # Send throttle command
    set_attitude_throttle(throttle_cmd, time_boot_ms)

    # Read airspeed
    airspeed = vehicle.airspeed if vehicle.airspeed is not None else np.nan
    airspeed_log.append([current_time, throttle_cmd, airspeed])

print("Throttle chirp sequence complete.")

# === Step 5: Save to CSV ===
airspeed_log = np.array(airspeed_log)
np.savetxt('chirp_airspeed_response_1to50Hz.csv', airspeed_log,
           delimiter=',', header='time,throttle,airspeed', comments='')

print("Saved airspeed log to 'chirp_airspeed_response_1to100Hz.csv'")

# === Step 6: Plot Input/Output ===
time_data = airspeed_log[:, 0]
throttle_data = airspeed_log[:, 1]
airspeed_data = airspeed_log[:, 2]

plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(time_data, throttle_data, label='Throttle Command')
plt.ylabel('Throttle')
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time_data, airspeed_data, label='Measured Airspeed', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Airspeed (m/s)')
plt.grid(True)
plt.legend()

plt.suptitle('Throttle Chirp Input (1–100 Hz) and Airspeed Response')
plt.tight_layout()
plt.show()

# === Step 7: Clean Up ===
vehicle.close()
print("Disconnected from vehicle.")
