from dronekit import connect, VehicleMode
import time
import random

def set_speed(vehicle, speed):
    print(f"Setting speed to {speed} m/s")
    vehicle.airspeed = speed

def fly_speed_loop():
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
    
    print("Reached target altitude. Now changing speeds every 0.5s.")
    
    speed_values = [14, 16, 20, 18, 22, 15]  # Some randomized speeds
    index = 0
    
    while True:
        set_speed(vehicle, 18)
        index += 1
        time.sleep(1)  # Change speed every 0.5 seconds
    
if __name__ == "__main__":
    fly_speed_loop()