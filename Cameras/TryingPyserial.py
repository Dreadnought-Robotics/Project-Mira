import serial
import subprocess

ser = serial.Serial('/dev/ttyACM0', 9600) 

while True:
    serial_info = ser.readline().decode('utf-8').strip()
    print("Received:", serial_info)


    if serial_info == '1':
        print("Launching ROS launch file or bash script...")      
        subprocess.run(['roslaunch', 'Cameras', 'Ckimaakichut.launch'])