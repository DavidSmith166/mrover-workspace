import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib import style

import sys
import numpy as np


SAMPLE_LENGTH = 5
style.use('fivethirtyeight')


class LivePlotter():
    '''
    This script is currently not as generic as it should be
    and needs to be refactored.
    '''
    def __init__(self):
        self.length = SAMPLE_LENGTH
        self.interval = 5
        self.log_path = sys.argv[1]
        self.gps_data = self.readCSV('gps', float, True)
        self.odom_data = self.readCSV('odom', float, True)
        self.movAvg_data = self.readCSV('movAvg', float, True)
        self.gps_window_start = 0
        self.gps_window_end = self.length
        self.odom__window_start = 0
        self.odom_window_end = self.length
        self.movAvg_window_start = 0
        self.movAvg_window_end = self.length
        self.gps_window = self.gps_data[:self.length]
        self.odom_window = self.odom_data[:self.length]
        self.movAvg_window = self.movAvg_data[:self.length]

    def readCSV(self, type, dtype, names_in):
        # Reads in the CSV file specified by log
        return np.genfromtxt(self.log_path + type + 'Log.csv',
                             delimiter=',',
                             names=True,
                             dtype=dtype)

    def run(self):
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        self.ax1.set_xlim([-.0002, .0002])
        self.ax1.set_ylim([-.0002, .0002])
        self.lines = [] * 3
        self.ani = animation.FuncAnimation(self.fig,
                                           self.animate,
                                           interval=self.interval,
                                           blit=True)

        plt.ion()
        plt.legend(labels=["Raw", "Filtered", "Moving Average"])
        plt.title('Live Comparison of Different Absolute Positioning Algorithms')
        plt.show()

    def shift_windows(self):
        self.gps_window_start += 1
        self.gps_window_end += 1
        self.odom__window_start += 1
        self.odom_window_end += 1
        self.movAvg_window_start += 1
        self.movAvg_window_end += 1
        # TODO bounds checking
        self.gps_window = self.gps_data[self.gps_window_start:self.gps_window_end]
        self.odom_window = self.odom_data[self.odom__window_start:self.odom_window_end]
        self.movAvg_window = self.movAvg_data[self.movAvg_window_start:self.movAvg_window_end]

    def animate(self, frame, *args):
        self.shift_windows()
        self.lines[0].set_data(self.gps_window)
        self.lines[1].set_data(self.odom_window)
        self.lines[2].set_data(self.movAvg_window)
        return self.lines


def main():
    liveplotter = LivePlotter()
    liveplotter.run()


if __name__ == '__main__':
    main()
