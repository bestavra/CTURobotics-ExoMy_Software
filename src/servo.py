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
    # If desired, you can change the names here (replace all names in this file only)
    S_ONE,S_TWO,S_THREE,S_FOUR = range(0, 4)

    def __init__(self):

        # Dictionary containing the pins of all servo
        self.pins = {
            'servo': {}
        }

        # PWM characteristics
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)  # Hz

        #Number at the end is the number of connected servos
        self.servo_pwm_neutral = [None] * 4
        self.servo_pwm_range = [None] * 4

        #Set servo pins:
        self.pins['servo'][self.S_ONE] = rospy.get_param("pin_servo_1")
        self.pins['servo'][self.S_TWO] = rospy.get_param("pin_servo_2")
        self.pins['servo'][self.S_THREE] = rospy.get_param("pin_servo_3")
        self.pins['servo'][self.S_FOUR] = rospy.get_param("pin_servo_4")
        
        # Set variables for the GPIO motor pins
        if rospy.get_param("pin_servo_1")  != 99:
            self.servo_pwm_neutral[self.S_ONE] = rospy.get_param("servo_pwm_neutral_1")
            self.servo_pwm_range[self.S_ONE] = rospy.get_param("servo_pwm_range_1")

        if rospy.get_param("pin_servo_2")  != 99:
            self.servo_pwm_neutral[self.S_TWO] = rospy.get_param("servo_pwm_neutral_2")
            self.servo_pwm_range[self.S_TWO] = rospy.get_param("servo_pwm_range_2")

        if rospy.get_param("pin_servo_3")  != 99:
            self.servo_pwm_neutral[self.S_THREE] = rospy.get_param("servo_pwm_neutral_3")
            self.servo_pwm_range[self.S_THREE] = rospy.get_param("servo_pwm_range_3")

        if rospy.get_param("pin_servo_4")  != 99:
            self.servo_pwm_neutral[self.S_FOUR] = rospy.get_param("servo_pwm_neutral_4")
            self.servo_pwm_range[self.S_FOUR] = rospy.get_param("servo_pwm_range_4")

        # Set steering motors to neutral values (straight)
        for servo_name, servo_pin in self.pins['servo'].items():
            if servo_pin != 99:
                self.pwm.set_pwm(servo_pin, 0, self.servo_pwm_neutral[servo_name])
                time.sleep(0.1)
            else:
                pass

    def setAngle(self, angle_command):
        # Loop through pin dictionary. The items key is the servo_name and the value the pin.
        for servo_name, servo_pin in self.pins['servo'].items():
            if servo_pin != 99:
                #90 degree limit protection (does not protect from false configuration of PWM settings)
                if angle_command >= -90 and angle_command <= 90:
                    duty_cycle = int(self.servo_pwm_neutral[servo_name] + angle_command[servo_name]/90.0 * self.servo_pwm_range[servo_name])
                    
                    self.pwm.set_pwm(servo_pin, 0, duty_cycle)
                else:
                    pass
            else:
                pass