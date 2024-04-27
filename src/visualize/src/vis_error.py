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
    forward_list.append(msg.x)
    lateral_list.append(msg.y)
    heave_error.append(msg.z)
    yaw_error.append(msg.w)


# Define a callback function to handle incoming messages
# def depth_yaw_callback(data):
#     # Append the received data to the list
#     data_list.append(data.external_pressure)

# def forward_callback(msg):
#     no_of_arucos = len(msg.data) // 9
#     y, x = 0, 0  # Swapped y and x to match the original code
#     for i in range(no_of_arucos):
#         x += (-1) * msg.data[i * 9 + 3]
#         y += (-1) * msg.data[i * 9 + 4]

#     forward_error = x / no_of_arucos
#     lateral_error = y / no_of_arucos
#     m = msg.data[3]
#     if not forward_list:
#         if forward_list:
#             it = forward_list[-1]
#             if it - forward_error > threshold:
#                 num_elements = min(len(forward_list), 10)
#                 max_element = max(forward_list[-num_elements:])
#                 forward_error = max_element
#             elif it - forward_error < -1 * threshold:
#                 num_elements = min(len(forward_list), 10)
#                 min_element = min(forward_list[-num_elements:])
#                 forward_error = min_element
#     forward_list.append(forward_error)
#     if not lateral_list:
#         if lateral_list:
#             it = lateral_list[-1]
#             if it - lateral_error > threshold:
#                 num_elements = min(len(lateral_list), 10)
#                 max_element = max(lateral_list[-num_elements:])
#                 lateral_error = max_element
#             elif it - lateral_error < -1 * threshold:
#                 num_elements = min(len(forward_list), 10)
#                 min_element = min(lateral_list[-num_elements:])
#                 lateral_error = min_element
#     lateral_list.append(lateral_error)


def denoise(data):
    """Denoise data using Fourier Transform."""
    if len(data) == 0:
        return np.array([])
    fft_data = np.fft.fft(data)
    fft_data[10:] = 0  
    return np.fft.ifft(fft_data).real

# Initialize the ROS node
rospy.init_node("visualizer", anonymous=True)

# Create a list to store the received data
data_list = []
forward_list = []
lateral_list = []
# # Subscribe to the ROS topic
# rospy.Subscriber("/master/telemetry", telemetry, depth_yaw_callback)
# rospy.Subscriber("/aruco/waypoints", Float32MultiArray, forward_callback)
rospy.Subscriber("/docking/errors", Quaternion, error_callback)
forward_pub = rospy.Publisher("/mira/forward", Float32MultiArray, queue_size=1)
lateral_pub = rospy.Publisher("/mira/lateral", Float32MultiArray, queue_size=1)


# Rate at which to update the plot (in Hz)
# rate = rospy.Rate(30)  # for example, 10 Hz

denoised_forward_list = []
denoised_lateral_list = []
i = 0
# Loop to keep the node running and plot the data
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
        denoised_forward = denoise(forward_list)
        denoised_forward_list.append(denoised_forward[-1])
        plt.plot(denoised_forward_list, label = "Forward Error")
        plt.xlabel("Time")
        plt.ylabel("Forward Error")
        plt.title("Forward Error over Time")
        plt.grid(True)  
        # plt.legend()
        plt.draw()

        plt.figure(3)
        denoised_lateral = denoise(lateral_list)
        denoised_lateral_list.append(denoised_lateral[-1])
        plt.plot(forward_list, label = "Lateral Error")
        plt.xlabel("Time")
        plt.ylabel("Lateral Error")
        plt.title("Lateral Error over Time")
        plt.grid(True)
        # plt.legend()
        # print(denoised_forward_list)
        plt.draw()
        f = Float32MultiArray()
        f.data = denoised_forward_list
        # l = Float32MultiArray()
        # l.data = denoised_lateral_list
        forward_pub.publish(f)
        i=i+1
        print(i)
        if i>100:
            i=0
            forward_list.clear()
        # lateral_pub.publish(l)
        plt.show(block=False)  # Show the plot without blocking the code execution
        plt.pause(0.0001)  # Pause for a short time to allow the plot to update
        # rate.sleep(   )
