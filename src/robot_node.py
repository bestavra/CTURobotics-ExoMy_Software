#!/usr/bin/env python
import time
from exomy.msg import Joystick, Commands, Screen
import rospy
from rover import Rover
import message_filters


global exomy
exomy = Rover()


def joy_callback(message):
    cmds = Commands()
    cmds.motor_angles = exomy.joystickToSteeringAngle(
        message.vel, message.steering)
    cmds.motor_speeds = exomy.joystickToVelocity(message.vel, message.steering)

    robot_pub.publish(cmds)


if __name__ == '__main__':
    rospy.init_node('robot')
    rospy.loginfo("Starting the robot node")
    global robot_pub
    joy_sub = rospy.Subscriber("/joystick", Joystick, joy_callback)

    rate = rospy.Rate(10)

    robot_pub = rospy.Publisher("/robot_commands", Commands, queue_size=1)
    screen_pub = rospy.Publisher("/screen", Screen, queue_size=1)

    rospy.spin()
