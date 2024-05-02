#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Quaternion
import numpy as np
from collections import deque    
from scipy import signal

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

rospy.init_node("error_denoiser", anonymous=True)
pub = rospy.Publisher("/docking/errors_denoise", Quaternion, queue_size=10)
rospy.Subscriber("/docking/errors", Quaternion, error_callback)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if len(surge_error) >= 20: 
        denoised_surge = lfilter_denoise(list(surge_error))
        denoised_sway = lfilter_denoise(list(sway_error))
        denoised_heave = lfilter_denoise(list(heave_error))
        denoised_yaw = lfilter_denoise(list(yaw_error))

        denoised_msg = Quaternion()
        denoised_msg.x = denoised_surge[-1]
        denoised_msg.y = denoised_sway[-1]
        denoised_msg.z = denoised_heave[-1]
        denoised_msg.w = denoised_yaw[-1]
        pub.publish(denoised_msg)

    rate.sleep()

rospy.spin()
