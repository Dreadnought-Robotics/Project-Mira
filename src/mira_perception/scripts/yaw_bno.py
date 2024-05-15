#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32
import serial

def main():
    rospy.init_node('arduino_yaw_publisher', anonymous=True)
    pub = rospy.Publisher('mira/heading', Int32, queue_size=10)
    # rate = rospy.Rate(100)  # 10hz

    ser = serial.Serial('/dev/ttyACM0', 115200)  # Update with your Arduino serial port

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                yaw_value = int(float(line))  # Read yaw value, convert to float, then to int
                rospy.loginfo(f"Yaw: {yaw_value}")
                pub.publish(yaw_value)
            except ValueError as e:
                rospy.logwarn(f"ValueError: {e} with line: {line}")
        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
