#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
import numpy as np
from custom_msgs.msg import telemetry

threshold = 8

surge_error = []
sway_error = []
heave_error = []
yaw_error = []


def error_callback(msg):
    global forward_list
    forward_list = msg.data
    # lateral_list.append(msg.y)
    # heave_error.append(msg.z)
    # yaw_error.append(msg.w)

def denoise(data):
    """Denoise data using Fourier Transform."""
    if len(data) == 0:
        return np.array([])
    fft_data = np.fft.fft(data)
    fft_data[10:] = 0  
    return np.fft.ifft(fft_data).real

# Initialize the ROS node
rospy.init_node("alizer", anonymous=True)

# Create a list to store the received data
data_list = []
global forward_list
forward_list = []
lateral_list = []
# # Subscribe to the ROS topic
# rospy.Subscriber("/master/telemetry", telemetry, depth_yaw_callback)
rospy.Subscriber("/mira/fixed_errors", Float32MultiArray, error_callback)
# rospy.Subscriber("/docking/errors", Quaternion, error_callback)
rate = rospy.Rate(10)  # for example, 10 Hz

denoised_forward_list = []
while not rospy.is_shutdown():
    # Plot the received data
    # plt.figure(1)
    # denoised_data = denoise(data_list)
    # plt.plot(denoised_data, label = "Data")
    # # plt.plot(data_list, label="Data")
    # plt.xlabel("Time")
    # plt.ylabel("Data")
    # plt.title("ROS Topic Data")
    # plt.grid(True)
    # # plt.legend()
    # plt.draw()
    # denoised_forward_mean = denoise(forward_list)
    # denoised_lateral_mean = denoise(lateral_list)
    # denoised_forward_mean_list.append(denoised_forward_mean)
    # denoised_lateral_mean_list.append(denoised_lateral_mean)
    if(len(forward_list)>0):
        plt.figure(2)
        denoised_forward = (forward_list)
        denoised_forward_list.append(denoised_forward[-1])
        plt.plot(denoised_forward_list, label = "Forward Error")
        plt.xlabel("Time")
        plt.ylabel("Forward Error")
        plt.title("Forward Error over Time")
        plt.grid(True)
        # plt.legend()
        plt.draw()

        # plt.figure(3)
        # denoised_lateral = denoise(lateral_list)
        # denoised_lateral_list.append(denoised_lateral[-1])
        # plt.plot(lateral_list, label = "Lateral Error")
        # plt.xlabel("Time")
        # plt.ylabel("Lateral Error")
        # plt.title("Lateral Error over Time")
        # plt.grid(True)
        # # plt.legend()
        # plt.draw()

        plt.show(block=False)  # Show the plot without blocking the code execution
        plt.pause(0.001)  # Pause for a short time to allow the plot to update
        rate.sleep()
