# Description:
# Sensor library for RaspberryPi
# Supporting:
# - Bosch BME280 temprature, pressure, and
# humidity sensor.
# - BH1750 light sensor.
# - ADS1015 analog-digital converter
# Author: Gernot Fattinger
# Date: 2019-02-02
# V1.2

import time
from smbus import SMBus

class bme280:

    def __init__(self, addr = 0x76, oversampling = 2):
        self.i2c = SMBus(1)
        self.addr = addr
        self.get_compensation_values()
        self.oversampling = oversampling

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

    def calc_delay(self, t_oversampling, h_oversampling, p_oversampling):
        t_delay = 0.000575 + 0.0023 * (1 << t_oversampling)
        h_delay = 0.000575 + 0.0023 * (1 << h_oversampling)
        p_delay = 0.001250 + 0.0023 * (1 << p_oversampling)
        return t_delay + h_delay + p_delay

    def get_adc_data(self):
        mode = 1  # forced
        t_oversampling = self.oversampling
        h_oversampling = self.oversampling
        p_oversampling = self.oversampling
        self.i2c.write_byte_data(self.addr, 0xF2, h_oversampling)  # ctrl_hum
        self.i2c.write_byte_data(self.addr, 0xF4, t_oversampling << 5 | p_oversampling << 2 | mode)  # ctrl
        delay = self.calc_delay(t_oversampling, h_oversampling, p_oversampling)
        time.sleep(delay)
        data = self.i2c.read_i2c_block_data(self.addr, 0xF7, 8)
        adc_T = (data[3]<<12) + (data[4]<<4) + ((data[5] & 0xF0)>>4)
        adc_P = (data[0]<<12) + (data[1]<<4) + ((data[2] & 0xF0)>>4)
        adc_H = (data[6]<<8) + data[7]
        return adc_T, adc_P, adc_H

    def convert_data(self, data):
        adc_T, adc_P, adc_H = data
        # Temperature conversion
        var1 = ((adc_T>>3)-(self.dig_T1<<1))*self.dig_T2>>11
        var2 = ( (((adc_T>>4)-self.dig_T1)*((adc_T>>4)-self.dig_T1)) >> 12 ) * self.dig_T3 >> 14
        self.t_fine = var1 + var2
        T =((self.t_fine*5+128)>>8)/100
        # Pressure conversion
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
        # Humidity conversion
        var1 = (self.t_fine-76800)
        var1 = (((((adc_H<<14)-(self.dig_H4<<20)-(self.dig_H5*var1))+16384)>>15)*((((((var1*self.dig_H6>>10)*((var1*self.dig_H3>>11)+32768))>>10)+2097152)*self.dig_H2+8192)>>14))
        var1 = (var1-(((((var1>>15)*(var1>>15))>>7)*self.dig_H1)>>4))
        var1 = 0 if var1 < 0 else var1
        var1 = 419430400 if var1 > 419430400 else var1
        rh = (var1>>12)/1024

        return T, p, rh

    def read(self):
        return self.convert_data(self.get_adc_data())


class bh1750:

    def __init__(self, addr=0x23):
        self.i2c = SMBus(1)
        self.addr = addr
        self.sens = 1.0
        self.sensitivity(self.sens)

    def convertToNumber(self, data):
        result=(data[1] + (256 * data[0])) / 1.2 / self.sens
        return (result)

    def read(self):
        data = self.i2c.read_i2c_block_data(self.addr, 0x20)
        return self.convertToNumber(data)

    def sensitivity(self, sens=None):
        if sens==None:
            return self.sens
        S = int(round(sens*69,0))
        S = max(min(S,254),31)
        LS = (S & 0x1F) | 0x60
        HS = ((S>>5) & 0x07) | 0x40
        self.i2c.write_byte(self.addr, LS)
        self.i2c.write_byte(self.addr, HS)
        self.sens = sens


class ads1015:

    def __init__(self, addr = 0x48):
        self.i2c = SMBus(1)
        self.addr = addr
        self.constants()
        self.mux = self.get(self.MUX)
        self.gain = self.get(self.GAIN)
        self.mode = self.get(self.MODE)
        self.data_rate = self.get(self.DATA_RATE)
        self.comp_mode = self.get(self.COMP_MODE)
        self.comp_pol = self.get(self.COMP_POL)
        self.comp_lat = self.get(self.COMP_LAT)
        self.comp_que = self.get(self.COMP_QUE)

    def constants(self):
        # Register adresses
        self.CONV_REG = 0b00
        self.CONFIG_REG = 0b01
        self.LO_TRESH_REG = 0b10
        self.HI_TRESH_REG = 0b11
        # Input MUX
        self.MUX = (12, 3, 'mux')
        self.MUX_DIFF_01 = 0b000
        self.MUX_DIFF_03 = 0b001
        self.MUX_DIFF_13 = 0b010
        self.MUX_DIFF_23 = 0b011
        self.MUX_0 = 0b100
        self.MUX_1 = 0b101
        self.MUX_2 = 0b110
        self.MUX_3 = 0b111
        # Gain
        self.GAIN = (9, 3, 'gain')
        self.GAIN_0 = 0b000  # +/-6.144V
        self.GAIN_1 = 0b001  # +/-4.096V
        self.GAIN_2 = 0b010  # +/-2.048V
        self.GAIN_3 = 0b011  # +/-1.024V
        self.GAIN_4 = 0b100  # +/-0.512V
        self.GAIN_5 = 0b101  # +/-0.256V
        self.VRANGE = [6.144, 4.096, 2.048, 1.024, 0.512, 0.256]
        # Conversion mode
        self.MODE = (8, 1, 'mode')
        self.MODE_CONTINUOUS = 0b0
        self.MODE_SINGLE_SHOT = 0b1
        # Data rate
        self.DATA_RATE = (5, 3, 'data_rate')
        self.DATA_RATE_128SPS = 0b000
        self.DATA_RATE_250SPS = 0b001
        self.DATA_RATE_490SPS = 0b010
        self.DATA_RATE_920SPS = 0b011
        self.DATA_RATE_1600SPS = 0b100
        self.DATA_RATE_2400SPS = 0b101
        self.DATA_RATE_3300SPS = 0b110
        # Comperator mode
        self.COMP_MODE = (4, 1, 'comp_mode')
        self.COMP_MODE_NORMAL = 0b0
        self.COMP_MODE_WINDOW = 0b1
        # Comperator polarity
        self.COMP_POL = (3, 1, 'comp_pol')
        self.COMP_POL_NORMAL = 0b0
        self.COMP_POL_INVERTED = 0b1
        # Comperator latching
        self.COMP_LAT = (2, 1, 'comp_lat')
        self.COMP_LAT_OFF = 0b0
        self.COMP_LAT_ON = 0b1
        # Comperator queue
        self.COMP_QUE = (0, 2, 'comp_que')
        self.COMP_QUE_1 = 0b00
        self.COMP_QUE_2 = 0b01
        self.COMP_QUE_4 = 0b10
        self.COMP_QUE_OFF =  0b11

    def bitwrite16(self, word, value, bitshift, bitwidth):
        return ((word & (0xFFFF - ((2**bitwidth-1)<<bitshift))) | (value<<bitshift))

    def byteswap(self, word):
        return ((word>>8) & 0x00FF) + ((word<<8) & 0xFF00)

    def get(self, parameter):
        config = self.byteswap(self.i2c.read_word_data(self.addr, self.CONFIG_REG))
        return (config>>parameter[0]) & (2**parameter[1]-1)

    def set(self, parameter, value):
        config = self.byteswap(self.i2c.read_word_data(self.addr, self.CONFIG_REG))
        config = self.bitwrite16(config, value, bitshift=parameter[0], bitwidth=parameter[1]) & 0x7FFF
        self.i2c.write_word_data(self.addr, self.CONFIG_REG, self.byteswap(config))
        setattr(self, parameter[2], value)

    def convert(self):
        config = self.byteswap(self.i2c.read_word_data(self.addr, self.CONFIG_REG))
        config = config | 0x8000
        self.i2c.write_word_data(self.addr, self.CONFIG_REG, self.byteswap(config))

    def isconverting(self):
        config = self.byteswap(self.i2c.read_word_data(self.addr, self.CONFIG_REG))
        return ((config & 0x8000)==0)

    def read(self, mux=None):
        if mux in [0,1,2,3,4,5,6,7]:
            self.set(self.MUX, mux)
        if self.mode==self.MODE_SINGLE_SHOT:
            self.convert()
            while self.isconverting():
                pass
        return self.byteswap(self.i2c.read_word_data(self.addr, self.CONV_REG))

    def voltage(self,mux=None):
        return self.read(mux)*self.VRANGE[self.gain]/0x7FFF

    def set_input(self, pin=None):
        if pin in [0,1,2,3]:
            self.set(self.MUX, pin+4)

    def set_gain(self, gain=None):
        if gain in [0,1,2,3,4,5]:
            self.set(self.GAIN, gain)

class veml6075:

    def __init__(self, addr = 0x10):
        self.i2c = SMBus(1)
        self.addr = addr
        self.i2c.write_byte_data(self.addr, 0x00, (0 & 7)<<4)
        self.a = 2.22
        self.c = 2.95
        self.b = 1.33
        self.d = 1.74
        self.UVAresp = 0.001461
        self.UVBresp = 0.002591
        self.UVA2Wm = 1/93
        self.UVB2Wm = 1/210

    def read(self):
        UVA = self.i2c.read_word_data(self.addr,0x07)
        UVB = self.i2c.read_word_data(self.addr,0x09)
        UVcomp1 = self.i2c.read_word_data(self.addr,0x0A)
        UVcomp2 = self.i2c.read_word_data(self.addr,0x0B)
        UVAcalc = UVA - (self.a*UVcomp1) - (self.b*UVcomp2)
        UVBcalc = UVB - (self.c*UVcomp1) - (self.d*UVcomp2)
        UVAI = UVAcalc * self.UVAresp
        UVBI = UVBcalc * self.UVBresp
        UVI = (UVAI + UVBI)/2
        UVAWm = UVAcalc * self.UVA2Wm
        UVBWm = UVBcalc * self.UVB2Wm
        return UVI
