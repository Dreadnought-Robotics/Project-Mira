import serial
import time

def send_data(data, duration):
    start_time = time.time()
    
    while time.time() - start_time < duration:
        ser.write(data.encode())
        time.sleep(0.1)  # Adjust the sleep duration as needed

# Replace '/dev/ttyUSB0' with the actual serial port name
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

try:
    # Send "0" for 30 seconds
    send_data("0", 30)

    # Send a single "1"
    ser.write("1".encode())

finally:
    ser.close()
    