#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
from collections import deque    
from scipy import signal

threshold = 8

surge_error = deque(maxlen=100)
sway_error = deque(maxlen=100)
heave_error = deque(maxlen=100)
yaw_error = deque(maxlen=100)

def error_callback(msg):
    surge_error.append(msg.x)
    sway_error.append(msg.y)
    heave_error.append(msg.z)
    yaw_error.append(msg.w)

def lfilter_denoise(data):
    if len(data) == 0:
        return np.array([])
    b, a = signal.butter(3, 0.05)
    y = signal.filtfilt(b, a, data)
    return y

def fft_denoise(data):
    if len(data) == 0:
        return np.array([])
    fft_data = np.fft.fft(data)
    fft_data[10:] = 0  
    return np.fft.ifft(fft_data).real
    
def savgol_filter(data):
    if len(data) == 0:
        return np.array([])
    if len(data) > 50:
        y = signal.savgol_filter(data, 50, 2)
        return y
    else :
        y = data
        return y

# def 
    

rospy.init_node("visualizer", anonymous=True)
rospy.Subscriber("/docking/errors/3d", Quaternion, error_callback)
rate = rospy.Rate(10)
fig, axs = plt.subplots(2, 2)
p = rospy.Publisher("/docking/errors/3d/refined", Quaternion, queue_size=10)
while not rospy.is_shutdown():
    if len(surge_error) >= 20: 
        q = Quaternion()
        denoised_surge = lfilter_denoise(list(surge_error))
        axs[0, 0].clear()
        axs[0, 0].plot(denoised_surge, label="Surge Error")
        axs[0, 0].set(xlabel='Time', ylabel='Surge Error')
        axs[0, 0].grid(True)
        axs[0, 0].set_title('Surge Error over Time')

        # Denoise sway error using FFT
        denoised_sway = lfilter_denoise(list(sway_error))
        # Plot sway error
        axs[0, 1].clear()
        axs[0, 1].plot(denoised_sway, label="Sway Error")
        axs[0, 1].set(xlabel='Time', ylabel='Sway Error')
        axs[0, 1].grid(True)
        axs[0, 1].set_title('Sway Error over Time')

        # Denoise heave error using FFT
        denoised_heave = lfilter_denoise(list(heave_error))
        # Plot heave error
        axs[1, 0].clear()
        axs[1, 0].plot(denoised_heave, label="Heave Error")
        axs[1, 0].set(xlabel='Time', ylabel='Heave Error')
        axs[1, 0].grid(True)
        axs[1, 0].set_title('Heave Error over Time')

        # Denoise yaw error using FFT
        denoised_yaw = lfilter_denoise(list(yaw_error))
        # Plot yaw error
        axs[1, 1].clear()
        axs[1, 1].plot(denoised_yaw, label="Yaw Error")
        axs[1, 1].set(xlabel='Time', ylabel='Yaw Error')
        axs[1, 1].grid(True)
        axs[1, 1].set_title('Yaw Error over Time')

        plt.pause(0.001)  # Pause for a short time to allow the plot to update
        q.x = -1*denoised_surge[-1]
        q.y = denoised_sway[-1]
        p.publish(q)
    rate.sleep()

plt.show()  # Show the plot