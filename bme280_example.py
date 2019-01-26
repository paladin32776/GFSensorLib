import time
from GFSensorLib import bme280

sensor = bme280()
while True:
    T,p,rh = sensor.read()
    print("%5.2f degC  %7.2f mbar  %6.2f %%" % (T,p,rh))
    time.sleep(1) 
