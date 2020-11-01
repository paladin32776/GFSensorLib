from GFSensorLib import lidarv4

lidar = lidarv4()

while True:
    distance = lidar.read()
    print(distance)
