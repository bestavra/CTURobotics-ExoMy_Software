#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import time
import numpy as np

import Adafruit_PCA9685


class Servo():
    '''
    Servo class contains all functions to control additional servos
    '''

    # Define servo names
    ONE,TWO = range(0, 2)
    #ONE, TWO = range(0,2)

    def __init__(self):

        # Dictionary containing the pins of all servo
        self.pins = {
            'servo': {}
        }

        # Set variables for the GPIO motor pins
        self.pins['servo'][self.ONE] = rospy.get_param("pin_servo_1")
        #self.pins['servo'][self.TWO] = rospy.get_param("pin_servo_2")

        # PWM characteristics
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)  # Hz

        #Number at the end is the number of connected servos
        self.servo_pwm_neutral = [None] * 1
        self.servo_pwm_range = [None] * 1

        self.servo_pwm_neutral[self.ONE] = rospy.get_param("servo_pwm_neutral_1")
        self.servo_pwm_range[self.ONE] = rospy.get_param("servo_pwm_range_1")
        #self.servo_pwm_neutral[self.TWO] = rospy.get_param("servo_pwm_neutral_2")
        #self.servo_pwm_range[self.TWO] = rospy.get_param("servo_pwm_range_2")

        # Set steering motors to neutral values (straight)
        for servo_name, servo_pin in self.pins['servo'].items():
            self.pwm.set_pwm(servo_pin, 0,
                             self.servo_pwm_neutral[servo_name])
            time.sleep(0.1)

    def setAngle(self, steering_command):
        # Loop through pin dictionary. The items key is the wheel_name and the value the pin.
        for servo_name, servo_pin in self.pins['servo'].items():
            duty_cycle = int(
                self.servo_pwm_neutral[servo_name] + steering_command[servo_name]/90.0 * self.servo_pwm_range[servo_name])

            self.pwm.set_pwm(servo_pin, 0, duty_cycle)