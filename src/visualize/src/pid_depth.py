#!/usr/bin/python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from custom_msgs.msg import telemetry

error = []
setpoint = 1050

def visual_depth_callback(msg):
    error.append(setpoint - msg.external_pressure)

if __name__ == "__main__":
    rospy.init_node("visualizing depth pid")
    rospy.Subscriber("/master/telemetry", telemetry, visual_depth_callback)
    