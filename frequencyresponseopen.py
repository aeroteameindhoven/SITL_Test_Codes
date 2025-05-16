import numpy as np
import time
import matplotlib.pyplot as plt
from dronekit import connect

# === Step 1: Load PRBS signal from CSV ===
prbs_data = np.loadtxt('1to100PRBS0.0001TS.csv', delimiter=',')
time_stamps = prbs_data[:, 0]
throttle_values = prbs_data[:, 1]

# === Step 2: Connect to SITL using DroneKit ===
print("Connecting to SITL using DroneKit...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
master = vehicle._master  # Access pymavlink connection
print("Connected to vehicle.")

# === Step 3: Define MAVLink throttle-only SET_ATTITUDE_TARGET ===
def set_attitude_throttle(throttle, time_boot_ms):
    master.mav.set_attitude_target_send(
        time_boot_ms,                    # time_boot_ms (uint32, in ms)
        master.target_system,
        master.target_component,
        0b00000111,                      # Ignore orientation, use throttle only
        [0, 0, 0, 0],                    # Quaternion (ignored)
        0, 0, 0,                         # Angular rates (ignored)
        throttle                         # Throttle (0.0 to 1.0)
    )

# === Step 4: Send PRBS signal and record airspeed ===
airspeed_log = []
start_time = time.time()

print("Sending PRBS throttle signal...")
for i in range(len(time_stamps)):
    target_time = time_stamps[i]
    throttle_cmd = throttle_values[i]

    # Wait for real time to match scheduled time
    while (time.time() - start_time) < target_time:
        time.sleep(0.00005)

    current_time = time.time() - start_time
    time_boot_ms = int(current_time * 1000)  # convert to milliseconds

    # Send throttle command
    set_attitude_throttle(throttle_cmd, time_boot_ms)

    # Record airspeed
    airspeed = vehicle.airspeed if vehicle.airspeed is not None else np.nan
    airspeed_log.append([current_time, throttle_cmd, airspeed])

print("Throttle sequence complete.")

# === Step 5: Save log to CSV ===
airspeed_log = np.array(airspeed_log)
np.savetxt('prbs_airspeed_response.csv', airspeed_log,
           delimiter=',', header='time,throttle,airspeed', comments='')

print("Saved airspeed log to 'prbs_airspeed_response.csv'")

# === Step 6: Plot response ===
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

plt.suptitle('Throttle PRBS Input and Airspeed Response')
plt.tight_layout()
plt.show()

# === Step 7: Clean up ===
vehicle.close()
print("Disconnected from vehicle.")
