#!/usr/bin/env python3

from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from scipy import signal
from std_msgs.msg import Float32, Float32MultiArray

threshold = 8

surge_error = deque(maxlen=250)
sway_error = deque(maxlen=250)
heave_error = deque(maxlen=250)
yaw_error = deque(maxlen=250)


def error_callback(msg):
    surge_error.append(msg.x)
    sway_error.append(msg.y)
    heave_error.append(msg.z)
    yaw_error.append(msg.w)


def lfilter_denoise(data):
    b, a = signal.butter(3, 0.05)
    y = signal.filtfilt(b, a, data)
    return y


def fft_denoise(data):
    if len(data) == 0:
        return np.array([])
    fft_data = np.fft.fft(data)
    fft_data[10:] = 0
    return np.fft.ifft(fft_data).real


rospy.init_node("visualizer", anonymous=True)
rospy.Subscriber("/docking/errors", Quaternion, error_callback)
rate = rospy.Rate(10)
fig, axs = plt.subplots(2, 2)

while not rospy.is_shutdown():
    if len(surge_error) >= 20:
        denoised_surge = lfilter_denoise(list(surge_error))
        axs[0, 0].clear()
        axs[0, 0].plot(denoised_surge, label="Surge Error")
        axs[0, 0].set(xlabel="Time", ylabel="Surge Error")
        axs[0, 0].grid(True)
        axs[0, 0].set_title("Surge Error over Time")

        # Denoise sway error using FFT
        denoised_sway = lfilter_denoise(list(sway_error))
        # Plot sway error
        axs[0, 1].clear()
        axs[0, 1].plot(denoised_sway, label="Sway Error")
        axs[0, 1].set(xlabel="Time", ylabel="Sway Error")
        axs[0, 1].grid(True)
        axs[0, 1].set_title("Sway Error over Time")

        # Denoise heave error using FFT
        denoised_heave = lfilter_denoise(list(heave_error))
        # Plot heave error
        axs[1, 0].clear()
        axs[1, 0].plot(denoised_heave, label="Heave Error")
        axs[1, 0].set(xlabel="Time", ylabel="Heave Error")
        axs[1, 0].grid(True)
        axs[1, 0].set_title("Heave Error over Time")

        # Denoise yaw error using FFT
        denoised_yaw = lfilter_denoise(list(yaw_error))
        # Plot yaw error
        axs[1, 1].clear()
        axs[1, 1].plot(denoised_yaw, label="Yaw Error")
        axs[1, 1].set(xlabel="Time", ylabel="Yaw Error")
        axs[1, 1].grid(True)
        axs[1, 1].set_title("Yaw Error over Time")

        plt.pause(0.001)  # Pause for a short time to allow the plot to update

    rate.sleep()

plt.show()  # Show the plot
