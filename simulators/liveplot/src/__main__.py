import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib import style

import sys
import numpy as np


SAMPLE_LENGTH = 10
style.use('fivethirtyeight')


class LivePlotter():
    '''
    This script is currently not as generic as it should be
    and needs to be refactored.
    '''
    def __init__(self):
        self.length = SAMPLE_LENGTH
        self.interval = 1
        self.log_path = sys.argv[1]
        self.gps_data = self.readCSV('gps', float, True)
        self.odom_data = self.readCSV('odom', float, True)
        self.movAvg_data = self.readCSV('movAvg', float, True)
        self.gps_data = {'long_deg': self.gps_data['long_deg'],
                         'long_min': self.gps_data['long_min'],
                         'lat_deg': self.gps_data['lat_deg'],
                         'lat_min': self.gps_data['lat_min']}
        self.odom_data = {'long_deg': self.odom_data['long_deg'],
                          'long_min': self.odom_data['long_min'],
                          'lat_deg': self.odom_data['lat_deg'],
                          'lat_min': self.odom_data['lat_min']}
        self.movAvg_data = {'long_deg': self.movAvg_data['long_deg'],
                            'long_min': self.movAvg_data['long_min'],
                            'lat_deg': self.movAvg_data['lat_deg'],
                            'lat_min': self.movAvg_data['lat_min']}
        self.gps_data['lon'] = [degs + mins for degs, mins in
                                zip(self.gps_data['long_deg'], self.gps_data['long_min'])]
        self.gps_data['lat'] = [degs + mins for degs, mins in
                                zip(self.gps_data['lat_deg'], self.gps_data['lat_min'])]
        self.odom_data['lon'] = [degs + mins for degs, mins in
                                 zip(self.odom_data['long_deg'], self.gps_data['long_min'])]
        self.odom_data['lat'] = [degs + mins for degs, mins in
                                 zip(self.odom_data['lat_deg'], self.gps_data['lat_min'])]
        self.movAvg_data['lon'] = [degs + mins for degs, mins in
                                   zip(self.movAvg_data['long_deg'], self.gps_data['long_min'])]
        self.movAvg_data['lat'] = [degs + mins for degs, mins in
                                   zip(self.movAvg_data['lat_deg'], self.gps_data['lat_min'])]
        print(self.gps_data)
        self.gps_window_start = 0
        self.gps_window_end = self.length
        self.odom_window_start = 0
        self.odom_window_end = self.length
        self.movAvg_window_start = 0
        self.movAvg_window_end = self.length
        self.gps_window = [self.gps_data['lon'][self.gps_window_start:self.gps_window_end],
                           self.gps_data['lat'][self.gps_window_start:self.gps_window_end]]
        self.odom_window = [self.odom_data['lon'][self.odom_window_start:self.odom_window_end],
                            self.odom_data['lat'][self.odom_window_start:self.odom_window_end]]
        self.movAvg_window = [self.movAvg_data['lon']
                              [self.movAvg_window_start:self.movAvg_window_end],
                              self.movAvg_data['lat']
                              [self.movAvg_window_start:self.movAvg_window_end]]

    def readCSV(self, type, dtype, names_in):
        # Reads in the CSV file specified by log
        return np.genfromtxt(self.log_path + type + 'Log.csv',
                             delimiter=',',
                             names=True,
                             dtype=dtype)

    def run(self):
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        self.ax1.set_xlim([-1, 1])
        self.ax1.set_ylim([-1, 1])
        self.lines = [self.ax1.plot([self.gps_window[0]],
                      [self.gps_window[1]], '-o', label='Raw')[0],
                      self.ax1.plot([self.odom_window[0]],
                      [self.odom_window[1]], '-b', label='Filtered')[0],
                      self.ax1.plot([self.movAvg_window[0]],
                      [self.movAvg_window[1]], '-og', label='Moving Average', linewidth=0.8)[0]]
        self.ani = animation.FuncAnimation(self.fig,
                                           self.animate,
                                           interval=self.interval,
                                           blit=False)
        plt.legend(labels=["Raw", "Filtered", "Moving Average"], handles=self.lines)
        plt.title('Live Comparison of Different Absolute Positioning Algorithms')
        plt.show()

    def shift_windows(self):
        self.gps_window_start += 1
        self.gps_window_end += 1
        self.odom_window_start += 1
        self.odom_window_end += 1
        self.movAvg_window_start += 1
        self.movAvg_window_end += 1
        # TODO bounds checking
        self.gps_window = [self.gps_data['lon'][self.gps_window_start:self.gps_window_end],
                           self.gps_data['lat'][self.gps_window_start:self.gps_window_end]]
        self.odom_window = [self.odom_data['lon'][self.odom_window_start:self.odom_window_end],
                            self.odom_data['lat'][self.odom_window_start:self.odom_window_end]]
        self.movAvg_window = [self.movAvg_data['lon']
                              [self.movAvg_window_start:self.movAvg_window_end],
                              self.movAvg_data['lat']
                              [self.movAvg_window_start:self.movAvg_window_end]]

    def animate(self, frame, *args):
        self.lines[0].set_data(self.gps_window)
        self.lines[1].set_data(self.odom_window)
        self.lines[2].set_data(self.movAvg_window)
        self.shift_windows()
        return [x for x in self.lines]


def main():
    liveplotter = LivePlotter()
    liveplotter.run()


if __name__ == '__main__':
    main()
