from pymavlink import mavutil

# Constant for conversion
G_TO_MS2 = 9.80665 / 1000  # 1 mG = 0.00980665 m/s^2

# Connect to SITL
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
master.wait_heartbeat()
print("✅ Connected to vehicle")

# Request RAW_IMU (ID 27) at 50 Hz
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    27,     # RAW_IMU message ID
    20000,  # 20 ms = 50 Hz
    0, 0, 0, 0, 0
)

# Read and convert IMU data
while True:
    msg = master.recv_match(type='RAW_IMU', blocking=True)
    
    # Convert from milli-g to m/s²
    ax = msg.xacc * G_TO_MS2
    ay = msg.yacc * G_TO_MS2
    az = msg.zacc * G_TO_MS2

    print(f"IMU: ax={ax:.3f} m/s² | ay={ay:.3f} m/s² | az={az:.3f} m/s²")
