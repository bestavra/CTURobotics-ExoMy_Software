#!/usr/bin/env python
# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#
# Merged and idea combination of:
# https://github.com/bchanudet/python-st7735s
# https://github.com/pimoroni/st7735-python
# This library is a modification of a modification of a modification of code originally written
# by Tony DiCola for Adafruit Industries, and modified to work with the ST7735 by Clement Skau.
#
# It has been modified by Pimoroni to include support for their 160x80 SPI LCD breakout, 
# and hopefully also generalised enough so that it will support other ST7735-powered displays.
# It was now merged with ST7735S code of bchanudet.
#
# It should work with ST7735S chips and displays
# It was tested on Raspberry Pi 4B with following pins:
# VLED on GPIO.23
# RST on GPIO.24
# A0/DC on GPIO.25
#
# It only has two main functions:
# ST7735s.display(image)
# ST7735s.fill(color)
# Image is a jpg or png compiled by PIL
# 
# Sample code:
# from py_st7735s import ST7735s
# from PIL import Image
# screen = ST7735s(port=0,cs=0, dc=22, rst=18, backlight=16, rotation=0, spi_speed_hz=16000000, 
#                  width=160, height=128, offset_left=0, offset_top=0, invert=False)
# image = Image.open('picture.png')
# image = image.resize((160, 128))
# screen.display(image)
# screen.close()

import numbers
import time
import numpy as np
import spidev
import RPi.GPIO as GPIO

__version__ = '0.0.4'

BG_SPI_CS_BACK = 0
BG_SPI_CS_FRONT = 1

SPI_CLOCK_HZ = 16000000

# Constants for interacting with display registers.
ST7735_TFTWIDTH = 128
ST7735_TFTHEIGHT = 160

ST7735_COLS = 128
ST7735_ROWS = 160

ST7735_NOP = 0x00
ST7735_SWRESET = 0x01
ST7735_RDDID = 0x04
ST7735_RDDST = 0x09

ST7735_SLPIN = 0x10
ST7735_SLPOUT = 0x11
ST7735_PTLON = 0x12
ST7735_NORON = 0x13

ST7735_INVOFF = 0x20
ST7735_INVON = 0x21
ST7735_DISPOFF = 0x28
ST7735_DISPON = 0x29

ST7735_CASET = 0x2A
ST7735_RASET = 0x2B
ST7735_RAMWR = 0x2C
ST7735_RAMRD = 0x2E

ST7735_PTLAR = 0x30
ST7735_MADCTL = 0x36
ST7735_COLMOD = 0x3A

ST7735_FRMCT1 = 0xB1
ST7735_FRMCT2 = 0xB2
ST7735_FRMCT3 = 0xB3
ST7735_INVCTR = 0xB4
ST7735_DISSET = 0xB6

ST7735_PWRCT1 = 0xC0
ST7735_PWRCT2 = 0xC1
ST7735_PWRCT3 = 0xC2
ST7735_PWRCT4 = 0xC3
ST7735_PWRCT5 = 0xC4
ST7735_VMCTR1 = 0xC5

ST7735_GAMCTP = 0xE0
ST7735_GAMCTN = 0xE1

ST7735_CMTEST = 0xF0
ST7735_PWRDIS = 0xF6

# Colours for convenience
ST7735_BLACK = 0x0000  # 0b 00000 000000 00000
ST7735_BLUE = 0x001F  # 0b 00000 000000 11111
ST7735_GREEN = 0x07E0  # 0b 00000 111111 00000
ST7735_RED = 0xF800  # 0b 11111 000000 00000
ST7735_CYAN = 0x07FF  # 0b 00000 111111 11111
ST7735_MAGENTA = 0xF81F  # 0b 11111 000000 11111
ST7735_YELLOW = 0xFFE0  # 0b 11111 111111 00000
ST7735_WHITE = 0xFFFF  # 0b 11111 111111 11111


def color565(r, g, b):
    """Convert red, green, blue components to a 16-bit 565 RGB value. Components
    should be values 0 to 255.
    """
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

def image_to_data(image, rotation=0):
    """Generator function to convert a PIL image to 16-bit 565 RGB bytes."""
    # NumPy is much faster at doing this. NumPy code provided by:
    # Keith (https://www.blogger.com/profile/02555547344016007163)
    pb = np.rot90(np.array(image.convert('RGB')), rotation // 90).astype('uint16')
    color = ((pb[:, :, 0] & 0xF8) << 8) | ((pb[:, :, 1] & 0xFC) << 3) | (pb[:, :, 2] >> 3)
    return np.dstack(((color >> 8) & 0xFF, color & 0xFF)).flatten().tolist()

class ST7735s(object):
    """Representation of an ST7735 TFT LCD."""

    def __init__(self, port=0, cs=0, dc=18, backlight=None, rst=None, width=ST7735_TFTWIDTH,
                 height=ST7735_TFTHEIGHT, rotation=90, offset_left=None, offset_top=None, invert=True, spi_speed_hz=4000000):
        """Create an instance of the display using SPI communication.

        Must provide the GPIO pin number for the D/C pin and the SPI driver.

        Can optionally provide the GPIO pin number for the reset pin as the rst parameter.

        :param port: SPI port number
        :param cs: SPI chip-select number (0 or 1 for BCM)
        :param backlight: Pin for controlling backlight
        :param rst: Reset pin for ST7735
        :param width: Width of display connected to ST7735
        :param height: Height of display connected to ST7735
        :param rotation: Rotation of display connected to ST7735
        :param offset_left: COL offset in ST7735 memory
        :param offset_top: ROW offset in ST7735 memory
        :param invert: Invert display
        :param spi_speed_hz: SPI speed (in Hz)

        """
        
        self._dc = dc
        self._rst = rst
        self._width = width
        self._height = height
        self._rotation = rotation
        self._invert = invert
        self._bitsPerPixel = 16
                
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        # Set DC as output.
        GPIO.setup(dc, GPIO.OUT)
        
        # Setup reset as output (if provided).
        if rst is not None:
            GPIO.setup(rst, GPIO.OUT)
        
        self._spi = spidev.SpiDev()
        self._spi.open(0, 0)
        self._spi.max_speed_hz = spi_speed_hz
        self._spi.mode = 0x00
        #self._spi.lsbfirst = False        
        
        # Default left offset to center display
        if offset_left is None:
            offset_left = (ST7735_COLS - width) // 2

        self._offset_left = offset_left

        # Default top offset to center display
        if offset_top is None:
            offset_top = (ST7735_ROWS - height) // 2

        self._offset_top = offset_top

        # Setup backlight as output (if provided).
        self._backlight = backlight
        if backlight is not None:
            GPIO.setup(backlight, GPIO.OUT)
            GPIO.output(backlight, GPIO.LOW)
            time.sleep(0.1)
            GPIO.output(backlight, GPIO.HIGH)

        self.hardReset()
        self.reset()
        
    def sendCommand(self, command, *bytes):
        GPIO.output(self._dc, 0)
        self._spi.writebytes([command])

        if len(bytes) > 0:
            GPIO.output(self._dc, 1)
            self._spi.writebytes(list(bytes))

    def data(self, data, chunk_size=4096):
        """Write a byte or array of bytes to the display. Is_data parameter
        controls if byte should be interpreted as display data (True) or command
        data (False).  Chunk_size is an optional size of bytes to write in a
        single SPI transaction, with a default of 4096.
        """
        # Set DC low for command, high for data.
        self.sendCommand(ST7735_RAMWR)
        GPIO.output(self._dc, 1)
        
        # Convert scalar argument to list so either can be passed as parameter.
        if isinstance(data, numbers.Number):
            data = [data & 0xFF]
        # Write data a chunk at a time.
        for start in range(0, len(data), chunk_size):
            end = min(start + chunk_size, len(data))
            self._spi.xfer(data[start:end])

    def set_backlight(self, value):
        """Set the backlight on/off."""
        if self._backlight is not None:
            GPIO.output(self._backlight, value)

    @property
    def width(self):
        return self._width if self._rotation == 0 or self._rotation == 180 else self._height

    @property
    def height(self):
        return self._height if self._rotation == 0 or self._rotation == 180 else self._width
    
    def hardReset(self):
        """Reset the display, if reset pin is connected."""
        if self._rst is not None:
            GPIO.output(self._rst, 1)
            time.sleep(.2)
            GPIO.output(self._rst, 0)
            time.sleep(.2)
            GPIO.output(self._rst, 1)
            time.sleep(.5)

    def reset(self):
        #GPIO.output(self.PinLight, 0)
        self.sendCommand(ST7735_SWRESET)
        time.sleep(0.3)
        self.sendCommand(ST7735_DISPOFF)
        time.sleep(0.3)
        
        # Framerate
        self.sendCommand(ST7735_FRMCT1, 0x01, 0x2c, 0x2d)
        self.sendCommand(ST7735_FRMCT2, 0x01, 0x2c, 0x2d)
        self.sendCommand(ST7735_FRMCT3, 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d)

        # Inversion
        self.sendCommand(ST7735_INVCTR, 0x07)
            
        if self._invert:
            self.sendCommand(ST7735_INVON)   # Invert display
        else:
            self.sendCommand(ST7735_INVOFF)  # Don't invert display
        
        # Power sequence
        self.sendCommand(ST7735_PWRCT1, 0xA2, 0x02, 0x84)
        self.sendCommand(ST7735_PWRCT2, 0xC5)
        self.sendCommand(ST7735_PWRCT3, 0x0A, 0x00)
        self.sendCommand(ST7735_PWRCT4, 0x8A, 0x2A)
        self.sendCommand(ST7735_PWRCT5, 0x8A, 0xEE)

        # Vcom ?
        self.sendCommand(ST7735_VMCTR1, 0x0E)

        # gamma sequence
        self.sendCommand(ST7735_GAMCTP, 0x0f, 0x1a, 0x0f, 0x18, 0x2f, 0x28, 0x20, 0x22, 0x1f, 0x1b, 0x23, 0x37, 0x00, 0x07, 0x02, 0x10)
        self.sendCommand(ST7735_GAMCTN, 0x0f, 0x1b, 0x0f, 0x17, 0x33, 0x2c, 0x29, 0x2e, 0x30, 0x30, 0x39, 0x3f, 0x00, 0x07, 0x03, 0x10)
        
        # test command
        self.sendCommand(ST7735_CMTEST, 0x01)

        # disable power save
        self.sendCommand(ST7735_PWRDIS, 0x00)

        # mode 262k
        if self._bitsPerPixel == 18:
            self.sendCommand(ST7735_COLMOD, 0x06)
        else:
            self.sendCommand(ST7735_COLMOD, 0x05)
            
        #self.sendCommand(ST7735_MADCTL, 104)
        self.sendCommand(ST7735_MADCTL, 0x60)
        #self.sendCommand(ST7735_MADCTL, 0x6C)

        self.sendCommand(ST7735_SLPOUT)
        time.sleep(0.3)
        self.sendCommand(ST7735_DISPON)

    def close(self):
        self._spi.close()
        GPIO.cleanup()

    def set_window(self, x0=0, y0=0, x1=None, y1=None):
        """Set the pixel address window for proceeding drawing commands. x0 and
        x1 should define the minimum and maximum x pixel bounds.  y0 and y1
        should define the minimum and maximum y pixel bound.  If no parameters
        are specified the default will be to update the entire display from 0,0
        to width-1,height-1.
        """
        if x1 is None:
            x1 = self._width - 1

        if y1 is None:
            y1 = self._height - 1

        y0 += self._offset_top
        y1 += self._offset_top

        x0 += self._offset_left
        x1 += self._offset_left
        
        self.sendCommand(ST7735_CASET, 0, (x0 & 0xff) , 0, (x1 & 0xff))
        self.sendCommand(ST7735_RASET, 0, (y0 & 0xff) , 0, (y1 & 0xff))
        
    def display(self, image):
        """Write the provided image to the hardware.

        :param image: Should be RGB format and the same dimensions as the display hardware.

        """
        # Set address bounds to entire display.
        self.set_window()
        
        # Convert image to array of 16bit 565 RGB data bytes.
        # Unfortunate that this copy has to occur, but the SPI byte writing
        # function needs to take an array of bytes and PIL doesn't natively
        # store images in 16-bit 565 RGB format.
        #
        pixelbytes = list(image_to_data(image, self._rotation))
        # Write data to hardware.
        self.data(pixelbytes)
    
    def fill(self, color):
        self.set_window()
        # (c, c, c) x 1261 = 3783 < max spi buffer size (4096 bytes)
        
        spiData = color * (1261)
        
        self.sendCommand(ST7735_RAMWR)
        GPIO.output(self._dc, 1)

        for y in range(15):
            self._spi.writebytes(spiData)
        
