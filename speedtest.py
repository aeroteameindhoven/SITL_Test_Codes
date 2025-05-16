from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import random

def set_speed(vehicle, speed):
    print(f"Setting speed to {speed} m/s")
    vehicle.airspeed = speed

def fly_mission():
    # Connect to the Vehicle
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    
    print("Arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    
    print("Taking off to 100m...")
    vehicle.simple_takeoff(100)
    
    while vehicle.location.global_relative_frame.alt < 95:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}m")
        time.sleep(1)
    
    print("Reached target altitude.")
    
    print("Starting straight-line flight...")
    target_location = LocationGlobalRelative(47.398170, 8.545649, 100)
    vehicle.simple_goto(target_location, airspeed=14)
    
    speed_values = [14, 16, 20, 18, 22, 15]  # Some randomized speeds
    index = 0
    
    for _ in range(20):  # Adjust loop count based on desired flight duration
        set_speed(vehicle, speed_values[index % len(speed_values)])
        index += 1
        time.sleep(0.5)  # Change speed every 0.5 seconds
    
    print("Mission complete! Hovering at last position.")
    
    vehicle.close()

if __name__ == "__main__":
    fly_mission()