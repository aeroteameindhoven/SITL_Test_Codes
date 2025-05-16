from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection(
    'udpout:127.0.0.1:14551',
    source_system=17,
    source_component=1
)

lat = 47.397742
lon = 8.545594
alt = 10.0

start_time = time.time()

print("Sending heartbeat + beacon position...")

while True:
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

    for _ in range(10):
        time_boot_ms = int((time.time() - start_time) * 1000)
        mav.mav.set_position_target_global_int_send(
            time_boot_ms,
            1, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111110000,  # pos + vel
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            16.0, 0, 0,
            0, 0, 0,
            0, 0
        )
        time.sleep(0.1)
