from dronekit import connect
import sys

serial = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A906H62E-if00-port0'

# Connect to the vehicle
print("Connecting to vehicle...")
vehicle = connect(serial, wait_ready=True, baud=57600)
print("Connected!")

# Get firmware
firmware = str(vehicle.version)

# Check if Plane
if "Plane" in firmware:
    print(f"Detected firmware: {firmware}")
    print("Proceeding with Plane stuff...")
else:
    print(f"Error: This board is running '{firmware}', not ArduPlane.")
    vehicle.close()
    sys.exit(1)
    
# Print location
location = str(vehicle.location.global_frame)
print(f"Location: {location}")

vehicle.close()
