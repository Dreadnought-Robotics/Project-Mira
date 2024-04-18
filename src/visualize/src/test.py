#!usr/bin/env python3
from collections import deque

import matplotlib.pyplot as plt
from scipy.signal import savgol_filter


class Visualizer:
    def __init__(self):
        self.data = [deque(maxlen=100) for _ in range(4)]
        self.filtered_data = [deque(maxlen=100) for _ in range(4)]
        self.window_size = 5
        self.poly_order = 3
        plt.ion()
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(
            2, 2, figsize=(10, 8)
        )
        self.lines = []
        self.filtered_lines = []
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            (line1,) = ax.plot([], [], "b", label="Original Data")
            (line2,) = ax.plot([], [], "r", label="Filtered Data")
            self.lines.append(line1)
            self.filtered_lines.append(line2)
            ax.legend()

    def run(self):
        while True:
            errors = list(map(int, input().split()))
            for i, error in enumerate(errors):
                self.data[i].append(error)
                if len(self.data[i]) >= self.window_size:
                    filtered_point = savgol_filter(
                        list(self.data[i]), self.window_size, self.poly_order
                    )[-1]
                    self.filtered_data[i].append(filtered_point)
            self.update_plot()

    def update_plot(self):
        for i, (line1, line2, data, filtered_data) in enumerate(
            zip(self.lines, self.filtered_lines, self.data, self.filtered_data)
        ):
            line1.set_xdata(range(len(data)))
            line1.set_ydata(data)
            line2.set_xdata(range(len(filtered_data)))
            line2.set_ydata(filtered_data)
            ax = [self.ax1, self.ax2, self.ax3, self.ax4][i]
            ax.relim()
            ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == "__main__":
    plotter = Visualizer()
    plotter.run()
    plt.show()
