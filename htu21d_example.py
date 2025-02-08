from GFSensorLib import htu21d

sensor = htu21d()
while True:
    T,RH = sensor.read()
    print("%5.2f degC  %6.2f %%" % (T,RH))
