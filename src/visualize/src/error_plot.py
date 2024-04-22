#!usr/bin/env python3

from collections import deque

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Quaternion
from scipy.signal import savgol_filter

from custom_msgs.msg import telemetry


class Visualizer:
    def __init__(self):
        rospy.init_node("visualizer", anonymous=True)
        self.data = deque(maxlen=100)
        self.filtered_data = deque(maxlen=100)
        self.window_size = 5
        self.poly_order = 3

        rospy.Subscriber("/docking/errors", Quaternion, self.error_callback)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        (self.line1,) = self.ax.plot([], [], "b", label="Original Data")
        (self.line2,) = self.ax.plot([], [], "r", label="Filtered Data")
        self.ax.legend()

    def error_callback(self, msg):
        error = [msg.x]
        self.data.append(error) 
        print(msg.x, msg.y, msg.z)
        # if len(self.data) >= self.window_size:
        #     filtered_point = savgol_filter(
        #         list(self.data), self.window_size, self.poly_order
        #     )[-1]
        #     self.filtered_data.append(filtered_point)
        self.update_plot()

    def update_plot(self):
        self.line1.set_xdata(range(len(self.data)))
        self.line1.set_ydata(self.data)
        # self.line2.set_xdata(range(len(self.filtered_data)))
        # self.line2.set_ydata(self.filtered_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == "__main__":
    plotter = Visualizer()
    rospy.spin()
