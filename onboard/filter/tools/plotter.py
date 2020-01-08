import numpy
import os
import math
import sys
import matplotlib.pyplot as plot
#TODO integrate with simulator

def lat2meters(lat):
    return lat * (math.pi/180) * 6371000

def long2meters(long, lat):
    return lat2meters(long) * math.cos((math.pi / 180) * lat)

def meters2lat(meters):
    return (meters * 180) / (math.pi * 6371000)

def meters2long(meters, lat):
    return meters2lat(meters) / math.cos((math.pi/180) * lat)

class Plotter:

    def readCsv(self, type):
        # Reads in the CSV file specified by log

        path_self = os.path.abspath(os.path.dirname(__file__))
        file_path = os.path.join(path_self, '../logs/')

        # Read data
        self.data = numpy.genfromtxt(file_path + type + 'Log.csv',
                                     delimiter=',', names=True)

    def plotCoords(self, subplot_loc):
        # Plots the coordinates from data in the specified subplot

        # Convert DMS to decimal
        long = self.data['long_deg'] + self.data['long_min']/60
        lat = self.data['lat_deg'] + self.data['lat_min']/60

        mean_lat = numpy.mean(lat)

        # Convert to meters
        for i in range(len(long)):
            long[i] = long2meters(long[i], mean_lat)
            lat[i] = lat2meters(lat[i])
        # Calculate delta vectors
        delta_long = long - long2meters(83.7382, 42.277)
        delta_lat = lat - lat2meters(42.277)

        # Calculate error circles
        sigma_long = numpy.std(delta_long)
        sigma_lat = numpy.std(delta_lat)
        cep = 0.56*sigma_long + 0.62*sigma_lat
        _2drms = 2*math.sqrt(sigma_long*sigma_long + sigma_lat*sigma_lat)

        # Plot
        plot.subplot(subplot_loc[0], subplot_loc[1], subplot_loc[2])
        plot.scatter(delta_long, delta_lat)
        cep_plot = plot.Circle((0, 0), radius=cep, color='r', fill=False)
        cep_label = plot.gca().add_patch(cep_plot)
        _2drms_plot = plot.Circle((0, 0), radius=_2drms, color='g', fill=False)
        _2drms_label = plot.gca().add_patch(_2drms_plot)
        _min = min(numpy.amin(delta_long)-sigma_long,
                   numpy.amin(delta_lat)-sigma_lat, -_2drms)
        _max = max(numpy.amax(delta_long)+sigma_long,
                   numpy.amax(delta_lat)+sigma_long, _2drms)
        # May need to change tolerance
        if abs(_min - _max) < 0.000000001:
            _min -= 1
            _max += 1
        plot.axis([_min, _max, _min, _max])
        plot.xticks(rotation=60)
        plot.xlabel('Delta Longitude (meters)')
        plot.ylabel('Delta Latitude (meters)')
        plot.title('GPS Coordinates')
        subplot = plot.subplot(1, 2, 1)
        subplot.text(0.5, -0.15, 'Long-Variance: ' + str(numpy.var(long)) +
                  '\nLat-Variance ' + str(numpy.var(lat)) + '\nLong-Mean: ' +
                  str(meters2long(numpy.mean(long), 42.277)) + '\nLat-Man: ' +
                  str(meters2lat(numpy.mean(lat))), ha='center', transform=subplot.transAxes)
        plot.legend(handles=[cep_label, _2drms_label], labels=['CEP', '2DRMS'])

    def plotSpeed(self, subplot_loc):
        # Plots the speed from data in the specified subplot

        # Plot
        plot.subplot(subplot_loc[0], subplot_loc[1], subplot_loc[2])
        plot.plot(range(0, 5*self.data.shape[0], 5), self.data['speed'])
        plot.axis([0, 5*self.data.shape[0], numpy.amin(self.data['speed']),
                  numpy.amax(self.data['speed'])])
        plot.xlabel('Time (seconds)')
        plot.ylabel('Speed (m/s)')
        plot.title('Speed')

    def plotBearing(self, subplot_loc):
        # Plots the bearing from data in the specified subplot

        # Plot
        plot.subplot(subplot_loc[0], subplot_loc[1], subplot_loc[2])
        plot.plot(range(0, 5*self.data.shape[0], 5), self.data['bearing'])
        plot.axis([0, 5*self.data.shape[0], numpy.amin(self.data['bearing']),
                  numpy.amax(self.data['bearing'])])
        plot.xlabel('Time (seconds)')
        plot.ylabel('Bearing (degrees)')
        plot.title('Bearing')

    def plot(self, data_type):
        # Decides what to plot
        if data_type == 'gps':
            self.readCsv('gps')
            self.plotCoords([1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        elif data_type == 'phone':
            self.readCsv('phone')
            self.plotCoords([1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        # elif data_type == 'imu':
            # self.plotImu()
        elif data_type == 'odom':
            self.readCsv('odom')
            self.plotCoords([1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        # elif data_type == 'all':
            # self.plotAll()
        elif data_type == 'test':
            self.readCsv('test')
            self.plotCoords([1, 2, 1])
            self.plotSpeed([2, 2, 2])
            self.plotBearing([2, 2, 4])
        else:
            print('Invalid data type.')
            sys.exit()

        plot.tight_layout()
        plot.suptitle(data_type)
        plot.show()


if __name__ == "__main__":
    # Get arguments
    if len(sys.argv) != 2:
        print('Error: Usage is py plotter.py <data_type>')
        sys.exit()

    # Plot
    plotter = Plotter()
    plotter.plot(sys.argv[1])