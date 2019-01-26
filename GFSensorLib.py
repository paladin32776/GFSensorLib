# Description:
# Sensor library for RaspberryPi
# Supporting:
# - Bosch BME280 temprature, pressure, and
# humidity sensor.
# - BH1750 light sensor.
# Author: Gernot Fattinger
# Date: 2019-01-26
# V1.1

from smbus import SMBus

class bme280:

    def __init__(self, addr = 0x76):
        self.i2c = SMBus(1)
        self.addr = addr
        self.get_compensation_values()

    def measuring(self):
        return ((self.i2c.read_byte_data(self.addr,0xF3) & 8) > 0)

    def nvm_updating(self):
        return ((self.i2c.read_byte_data(self.addr,0xF3) & 1) > 0)

    def int_to_signed_short(self,value):
        return -(value & 0x8000) | (value & 0x7fff)

    def int_to_signed_char(self,value):
        return -(value & 0x80) | (value & 0x7f)

    def read_unsigned_short(self,reg):
        return (self.i2c.read_byte_data(self.addr,reg+1)<<8) + self.i2c.read_byte_data(self.addr,reg)

    def read_signed_short(self,reg):
        return self.int_to_signed_short(self.read_unsigned_short(reg))

    def get_compensation_values(self):
        # Temperature compensation values
        self.dig_T1 = self.read_unsigned_short(0x88)
        self.dig_T2 = self.read_signed_short(0x8A)
        self.dig_T3 = self.read_signed_short(0x8C)
        # Pressure compensation values
        self.dig_P1 = self.read_unsigned_short(0x8E)
        self.dig_P2 = self.read_signed_short(0x90)
        self.dig_P3 = self.read_signed_short(0x92)
        self.dig_P4 = self.read_signed_short(0x94)
        self.dig_P5 = self.read_signed_short(0x96)
        self.dig_P6 = self.read_signed_short(0x98)
        self.dig_P7 = self.read_signed_short(0x9A)
        self.dig_P8 = self.read_signed_short(0x9C)
        self.dig_P9 = self.read_signed_short(0x9E)
        # Humidity compensation values
        self.dig_H1 = self.i2c.read_byte_data(self.addr,0xA1)
        self.dig_H2 = self.read_signed_short(0xE1)
        self.dig_H3 = self.i2c.read_byte_data(self.addr,0xE3)
        self.dig_H4 = self.int_to_signed_short((self.i2c.read_byte_data(self.addr,0xE4)<<4) + (self.i2c.read_byte_data(self.addr,0xE5) & 0x0F))
        self.dig_H5 = self.int_to_signed_short((self.i2c.read_byte_data(self.addr,0xE6)<<4) + ((self.i2c.read_byte_data(self.addr,0xE5)>>4) & 0x0F))
        self.dig_H6 = self.int_to_signed_char(self.i2c.read_byte_data(self.addr,0xE7))


    def get_temperature(self):
        adc_T = (self.i2c.read_byte_data(self.addr,0xFA)<<12) + (self.i2c.read_byte_data(self.addr,0xFB)<<4) + ((self.i2c.read_byte_data(self.addr,0xFC) & 0xF0)>>4)
        var1 = ((adc_T>>3)-(self.dig_T1<<1))*self.dig_T2>>11
        var2 = ( (((adc_T>>4)-self.dig_T1)*((adc_T>>4)-self.dig_T1)) >> 12 ) * self.dig_T3 >> 14
        self.t_fine = var1 + var2
        T =((self.t_fine*5+128)>>8)/100
        return T

    def get_pressure(self):
        adc_P = (self.i2c.read_byte_data(self.addr,0xF7)<<12) + (self.i2c.read_byte_data(self.addr,0xF8)<<4) + ((self.i2c.read_byte_data(self.addr,0xF9) & 0xF0)>>4)
        var1 = self.t_fine-128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + (var1*self.dig_P5<<17)
        var2 = var2 + (self.dig_P4<<35)
        var1 = (var1*var1*self.dig_P3>>8) + (var1*self.dig_P2<<12)
        var1 = ((1<<47)+var1)*self.dig_P1>>33
        if var1 == 0:
            p = 0
        else:
            p = 1048576-adc_P
            p = int(((p<<31)-var2)*3125/var1)
            var1 = self.dig_P9*(p>>13)*(p>>13) >> 25
            var2 = self.dig_P8*p >> 19
            p = ((p+var1+var2 >> 8)+(self.dig_P7 << 4))/25600
        return p

    def get_humidity(self):
        adc_H = (self.i2c.read_byte_data(self.addr,0xFD)<<8) + self.i2c.read_byte_data(self.addr,0xFE)
        v_x1_u32r = (self.t_fine-76800)
        v_x1_u32r = (((((adc_H<<14)-(self.dig_H4<<20)-(self.dig_H5*v_x1_u32r))+16384)>>15)*((((((v_x1_u32r*self.dig_H6>>10)*((v_x1_u32r*self.dig_H3>>11)+32768))>>10)+2097152)*self.dig_H2+8192)>>14))
        v_x1_u32r = (v_x1_u32r-(((((v_x1_u32r>>15)*(v_x1_u32r>>15))>>7)*self.dig_H1)>>4))
        v_x1_u32r = 0 if v_x1_u32r < 0 else v_x1_u32r
        v_x1_u32r = 419430400 if v_x1_u32r > 419430400 else v_x1_u32r
        rh = (v_x1_u32r>>12)/1024
        return rh

    def read(self):
        return self.get_temperature(), self.get_pressure(), self.get_humidity()


class bh1750:

    def __init__(self, addr=0x23):
        self.i2c = SMBus(1)
        self.addr = addr

    def convertToNumber(self, data):
        result=(data[1] + (256 * data[0])) / 1.2
        return (result)

    def read(self):
        data = self.i2c.read_i2c_block_data(self.addr, 0x20)
        return self.convertToNumber(data)
