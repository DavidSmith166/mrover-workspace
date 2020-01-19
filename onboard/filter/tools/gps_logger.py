import csv

def write(contents):
        # Writes contents to the log
        with open('altGpsLog.csv', mode='a') as log:
            writer = csv.writer(log)
            writer.writerow(contents)

def Logger():
    write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'speed', 'track_angle'])

    while True:
        lat_deg, lat_min, long_deg, long_min, speed, track_angle = yield
        write([lat_deg, lat_min, long_deg, long_min, speed, track_angle])
