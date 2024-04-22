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
<<<<<<< HEAD
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
=======
        error = [msg.x, msg.y, msg.z, msg.w]
        self.data.append(error)
        if len(self.data) >= self.window_size:
            filtered_point = savgol_filter(
                [d[0] for d in self.data], self.window_size, self.poly_order
            )[-1]
            self.filtered_data.append(filtered_point)

    def update_plot(self):
        self.line1.set_xdata(range(len(self.data)))
        self.line1.set_ydata([d[0] for d in self.data])
        self.line2.set_xdata(range(len(self.filtered_data)))
        self.line2.set_ydata(self.filtered_data)
>>>>>>> 5a87782266b7dbaad221cbdd0b29745436ca8c7a
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == "__main__":
    plotter = Visualizer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        plotter.update_plot()
        rate.sleep()

    rospy.spin()
