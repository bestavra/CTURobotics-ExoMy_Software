#!/usr/bin/env python
import time
from exomy.msg import RoverCommand, MotorCommands
from sensor_msgs.msg import Range
import rospy
import numpy as np
from rover import Rover
import message_filters


global exomy
exomy = Rover()

global range
range = 0

def joy_callback(message):
    cmds = MotorCommands()
    
    global range
    min_safety_range = 19
    max_safety_range = 45
    if range > -1 and range < max_safety_range and message.steering > 0 and message.locomotion_mode == 1:
        calc_range = np.clip(range,min_safety_range,max_safety_range)
        speed_factor = (max_safety_range - min_safety_range) * ( calc_range - min_safety_range) / 100
        message.vel = message.vel * speed_factor
        rospy.loginfo("Approching object...slowing down!")
    
    if message.motors_enabled is True:
        exomy.setLocomotionMode(message.locomotion_mode)

        cmds.motor_angles = exomy.joystickToSteeringAngle(
            message.vel, message.steering)
        cmds.motor_speeds = exomy.joystickToVelocity(
            message.vel, message.steering)
    else:
        cmds.motor_angles = exomy.joystickToSteeringAngle(0, 0)
        cmds.motor_speeds = exomy.joystickToVelocity(0, 0)

    robot_pub.publish(cmds)

def range_publisher(data):
    global range
    range = round(data.range*100,0)
    return

if __name__ == '__main__':
    rospy.init_node('robot_node')
    rospy.loginfo("Starting the robot node")
    global robot_pub
    joy_sub = rospy.Subscriber(
        "/rover_command", RoverCommand, joy_callback, queue_size=1)
    range_sub = rospy.Subscriber(
        "/range", Range, range_publisher, queue_size=1)
    
    
    rate = rospy.Rate(10)

    robot_pub = rospy.Publisher("/motor_commands", MotorCommands, queue_size=1)

    rospy.spin()
