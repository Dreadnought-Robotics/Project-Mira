#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32MultiArray

aruco_set = set()


def collecterCallback(msg):
    global aruco_set
    no_of_arucos = len(msg.data) // 1

    if no_of_arucos == 1:
        aruco_id = msg.data[0]

        l = len(aruco_set)
        if aruco_id > 0 and aruco_id < 100:
            aruco_set.add(aruco_id)

        if len(aruco_set) > l:
            rospy.loginfo(f"Aruco ID Detected {aruco_id}")


if __name__ == "__main__":
    rospy.init_node("arucocollecter")
    rospy.Subscriber("/aruco/pixels", Float32MultiArray, collecterCallback)
    rospy.spin()
