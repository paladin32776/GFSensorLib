from GFSensorLib import ads1015

adc = ads1015()
adc.set_input(0)
adc.set_gain(1)
while True:
    voltage = adc.voltage()
    print("%5.3f V" % (voltage))
