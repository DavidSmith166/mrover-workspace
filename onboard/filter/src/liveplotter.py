import matplotlib.pyplot as plt
# import matplotlib.animation as animation
from matplotlib import style
from enum import IntEnum
from rover_common import aiolcm
from rover_msgs import GPS, Odometry
# \ SensorPackage
import asyncio

# This should be moved to a config
SAMPLE_LENGTH = 5
style.use('fivethirtyeight')
# This should be moved to a config


class DataTypes(IntEnum):
    GPS = 0
    ODOM = 1
    # AVG = 2
    # PHONE = 3


class LivePlotter():
    '''
    Class holding methods and members relevant to the live plotting of the
    Rover's path based on odometry and sensor logs

    Note for positioning inside a tuple the ordering will always be (lon, lat)
    which corresponds to (x, y) in the cartesian plane

    This plotter is only intended for gps data (or other absolute position methods)
    '''
    def __init__(self):
        self.initialized = [False] * len(DataTypes)
        self.length = SAMPLE_LENGTH
        self.data = {}
        # Update interval in miliseconds
        self.interval = 5
        # Subscribe to LCM channels
        self.lcm = aiolcm.AsyncLCM()
        self.lcm.subscribe("/gps", self.gps_callback)
        # self.lcm.subscribe("/sensor_package", self.phone_callback)
        self.lcm.subscribe("/odometry", self.odom_callback)
        # Temp mov_avg filter
        # self.lcm.subscribe("/mov_avg", self.mov_avg_callback)

    async def run(self):
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        self.lines = [line for line, in [self.ax1.plot([], [], lw=3)] * len(DataTypes)]
        # self.ani = animation.FuncAnimation(self.fig,
        #                                    self.animate,
        #                                    interval=self.interval,
        #                                    blit=True)
        plt.ion()
        plt.legend(labels=[t.name for t in DataTypes])
        while True:
            # Arbitrary time
            if all(self.initialized):
                for t in DataTypes:
                    plt.scatter(*self.data[t], label=t.name)
            else:
                print(self.initialized)
            # plt.draw()
            plt.pause(0.001)
            await asyncio.sleep(0.001)

    def add_measurement(self, key, values):
        '''
        Appends the new measurement to the end of the array
        Removes the oldest measurement from the array

        Expects list (lon, lat) for values
        '''
        self.data[key][0].append(values[0])
        self.data[key][1].append(values[1])
        # self.data[key][0] = self.data[key][0][1:]
        # self.data[key][1] = self.data[key][1][1:]

    def animate(self, frame, *args):
        # self.ax1.clear()
        if all(self.initialized):
            for i, t in enumerate(DataTypes):
                self.lines[i].set_data(*self.data[t])
        return self.lines

    def gps_callback(self, channel, msg):
        gps = GPS.decode(msg)
        print(gps)
        lon = gps.longitude_deg + gps.longitude_min / 60
        lat = gps.latitude_deg + gps.latitude_min / 60
        print('GPS', lon, lat)
        if all(self.initialized):
            self.add_measurement(DataTypes.GPS, (lon, lat))
        else:
            self.data[DataTypes.GPS] = [[lon] * self.length, [lat] * self.length]
            self.initialized[DataTypes.GPS] = True
            print('wack 1')

    # def phone_callback(self, channel, msg):
    #     phone = SensorPackage.decode(msg)
    #     lon = phone.longitude_deg + phone.longitude_min / 60
    #     lat = phone.latitude_deg + phone.latitude_min / 60
    #     if all(self.initialized):
    #         self.add_measurement(DataTypes.PHONE, (lon, lat))
    #     else:
    #         self.data[DataTypes.PHONE] = [[lon] * self.length, [lat] * self.length]
    #         self.initialized[DataTypes.PHONE] = True

    def odom_callback(self, channel, msg):
        odom = Odometry.decode(msg)
        print(odom)
        lon = odom.longitude_deg + odom.longitude_min / 60
        lat = odom.latitude_deg + odom.latitude_min / 60
        print('Filter', lon, lat)
        if all(self.initialized):
            self.add_measurement(DataTypes.ODOM, (lon, lat))
        else:
            self.data[DataTypes.ODOM] = [[lon] * self.length, [lat] * self.length]
            self.initialized[DataTypes.ODOM] = True
            print('wack 2')

    # def mov_avg_callback(self, channel, msg):
    #     avg = Odometry.decode(msg)
    #     print(avg)
    #     lon = avg.longitude_deg + avg.longitude_min / 60
    #     lat = avg.latitude_deg + avg.latitude_min / 60
    #     if all(self.initialized):
    #         self.add_measurement(DataTypes.AVG, (lon, lat))
    #     else:
    #         self.data[DataTypes.AVG] = [[lon] * self.length, [lat] * self.length]
    #         self.initialized[DataTypes.AVG] = True
    #         print('wack 3')


liveplotter = LivePlotter()
