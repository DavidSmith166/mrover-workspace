import random
import csv
import os

if __name__ == '__main__':
    path_self = os.path.abspath(os.path.dirname(__file__))
    file_path = os.path.join(path_self, 'logs/')

    with open(file_path + 'testLog.csv', 'w+') as log:
        writer = csv.writer(log)
        writer.writerow(['lat_deg', 'lat_min', 'long_deg',
                         'long_min', 'bearing', 'speed'])
        sample = 100
        for i in range(0, sample):
            lat_min = random.uniform(16.25, 16.75)
            long_min = random.uniform(43.25, 43.75)
            bearing = random.uniform(40, 160)
            speed = random.uniform(0, 15)
            writer.writerow([42, lat_min, -83, long_min, bearing, speed])
        for i in range(0, round(.5*sample)):
            lat_min = random.uniform(15, 18)
            long_min = random.uniform(42, 45)
            bearing = random.uniform(40, 160)
            speed = random.uniform(0, 15)
            writer.writerow([42, lat_min, -83, long_min, bearing, speed])
