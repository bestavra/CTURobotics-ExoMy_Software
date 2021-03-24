#!/usr/bin/env python
# vim: set fileencoding=UTF-8 :
# -*- coding: utf-8 -*-

# HMC5888L Magnetometer (Digital Compass) wrapper class
# Based on https://bitbucket.org/thinkbowl/i2clibraries/src/14683feb0f96,
# https://github.com/rm-hull/hmc5883l,
# https://github.com/innovationgarage/hmc5883l,
# and https://github.com/RigacciOrg/py-qmc5883l,
# but uses smbus and sets some different init
# params.
# For calibration see (code must be modified):
# https://github.com/RigacciOrg/py-qmc5883l/tree/master/calibration

"""
Python driver for the HMC5883L 3-Axis Magnetic Sensor.

Usage example:

  import py_hmc5883l
  sensor = py_hmc5883l.HMC5883L()
  m = sensor.get_magnet()
  print(m)

you will get three 12 bit signed integers, representing the values
of the magnetic sensor on axis X, Y and Z.
"""

__author__ = "0xD0M1M0"
__copyright__ = ""
__license__ = "GPLv3-or-later"
__email__ = "e-mail@email.com"
__version__ = "0.0.1"


import smbus
import math
import time
import sys

class HMC5883L:

    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1e, gauss=4.0, declination=(0,0)):
        self.bus = smbus.SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        #declination in degree and minutes / set_declination is just float degrees
        self._declination = (degrees + minutes / 60) * math.pi / 180
        self._calibration = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]]
    
        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.mode_continuous()

    def __del__(self):
        """Once finished using the sensor, switch to standby mode."""
        self.mode_standby()

    def mode_continuous(self):
        """Set the device in continuous read mode."""
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def mode_standby(self):
        """Set the device in standby mode."""
        self.bus.write_byte_data(self.address, 0x02, 0x10) # Standby

    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: 
            # Some values have reached an overflow.
            msg = ("Magnetic sensor overflow.")
            msg += " Consider switching to a higher gauss output range."
            logging.warning(msg)
            return None
        return round(val * self.__scale, 4)

    def get_data(self):
        [x, y, z] = [None, None, None]
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        #print map(hex, data)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        return [x, y, z]
        
    def get_magnet_raw(self):
        """Get the 3 axis values from magnetic sensor."""
        [x, y, z] = self.get_data()
        return [x, y, z]

    def get_magnet(self):
        """Return the horizontal magnetic sensor vector with (x, y) calibration applied."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            [x1, y1] = [x, y]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + c[1][2]
        return [x1, y1]

    def get_bearing_raw(self):
        """Horizontal bearing (in degrees) from magnetic value X and Y."""
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            return b

    def get_bearing(self):
        """Horizontal bearing, adjusted by calibration and declination."""
        [x, y] = self.get_magnet()
        if x is None or y is None:
            return None
        else:
            b = math.degrees(math.atan2(y, x))
            if b < 0:
                b += 360.0
            b += self._declination
            if b < 0.0:
                b += 360.0
            elif b >= 360.0:
                b -= 360.0
        return b        

    def set_declination(self, value):
        """Set the magnetic declination, in degrees."""
        try:
            d = float(value)
            if d < -180.0 or d > 180.0:
                logging.error(u'Declination must be >= -180 and <= 180.')
            else:
                self._declination = d
        except:
            logging.error(u'Declination must be a float value.')

    def get_declination(self):
        """Return the current set value of magnetic declination."""
        return self._declination

    def set_calibration(self, value):
        """Set the 3x3 matrix for horizontal (x, y) magnetic vector calibration."""
        c = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        try:
            for i in range(0, 3):
                for j in range(0, 3):
                    c[i][j] = float(value[i][j])
            self._calibration = c
        except:
            logging.error(u'Calibration must be a 3x3 float matrix.')
            
    def get_calibration(self):
        """Return the current set value of the calibration matrix."""
        return self._calibration

    declination = property(fget=get_declination,
                           fset=set_declination,
                           doc=u'Magnetic declination to adjust bearing.')

    calibration = property(fget=get_calibration,
                           fset=set_calibration,
                           doc=u'Transformation matrix to adjust (x, y) magnetic vector.')