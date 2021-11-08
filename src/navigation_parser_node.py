#!/usr/bin/env python
import rospy
from exomy.msg import RoverCommand
from exomy.msg import NavigationCommand
from exomy.msg import Compass
from locomotion_modes import LocomotionMode
import math
import numpy as np

# Define locomotion modes
global locomotion_mode
global auto_navigation
#global motors_enabled

global bearing
bearing = 0

#locomotion_mode = LocomotionMode.ACKERMANN.value
#motors_enabled = True

def main_naviation(data):

    #global locomotion_mode
    #global motors_enabled
    global bearing
    global auto_navigation

    rover_cmd = RoverCommand()

    # Reading out navigation data
    heading = data.heading
    rospy.loginfo("Heading: " + str(heading))
    auto_navigation = data.auto_navigation
    rospy.loginfo("Auto navigation set to: " + str(auto_navigation))
    
    # Enable auto navigation
    if(auto_navigation is True):
        rospy.loginfo("LocomotionMode for auto navigation: Point Turn!")
        rover_cmd.locomotion_mode = LocomotionMode.POINT_TURN.value

        motors_enabled = True
        rospy.loginfo("Motors enabled for auto navigation!")
        rover_cmd.motors_enabled = motors_enabled

        # The velocity is decoded as value between 0...100
        #rover_cmd.vel = 30

        # The steering is described as an angle between -180...180
        # Which describe the joystick position as follows:
        #   +90
        # 0      +-180
        #   -90
        #
        
        #Define direction of turn
        diff_angle = ((heading - bearing) + 180 ) % 360 - 180
        rover_cmd.steering = steering_direction_diff_angle(diff_angle)

        # old and new speed are needed to stop the rover from sudden speed jumps if the compass is slighly inaccurate        
        # Define old and new speed with maximum velocity
        old_speed = 100
        speed = 100
        
        while(abs(diff_angle) > 2 and auto_navigation is True):
            minimum_speed = 3
            standard_speed = 25
            slow_down_angle = 60
            
            diff_angle = ((heading - bearing) + 180 ) % 360 - 180
            #rospy.loginfo("Diff: " + str(diff_angle))
            
            # Allow for direction change if difference angle was passed and return is more useful than a full turn
            if(abs(diff_angle) < 90):
                rover_cmd.steering = steering_direction_diff_angle(diff_angle)
            
            #Slow down
            if(abs(diff_angle) < slow_down_angle):
                # old/new speed comparison
                old_speed = speed
                speed = np.clip(round(abs(diff_angle)/slow_down_angle * standard_speed,0), minimum_speed, standard_speed)
                if (speed > old_speed): speed = old_speed
                rover_cmd.vel = speed
            else:
                rover_cmd.vel = standard_speed
       
            rover_cmd.connected = True

            pub.publish(rover_cmd)
        else:
            rospy.loginfo("Auto rotation angle difference reached: " + str(diff_angle))
            auto_navigation = False
            rover_cmd.vel = 0
            #rover_cmd.locomotion_mode = LocomotionMode.ACKERMANN.value
            pub.publish(rover_cmd)

def steering_direction_diff_angle(diff_angle):
    if(diff_angle > 0):
        steering = 180
    else:
        steering = 0
    return steering

def publish_compass_bearing(data):
    global bearing
    bearing = data.bearing
    #rospy.loginfo("Bearing: " + str(bearing))
    return

if __name__ == '__main__':
    global pub

    rospy.init_node('navigation_parser_node')
    rospy.loginfo('navigation_parser_node started')
    
    rate = rospy.Rate(10) # 10hz

    sub = rospy.Subscriber("/navigation_command", NavigationCommand, main_naviation, queue_size=1)
    compass_sub = rospy.Subscriber("/bearing", Compass, publish_compass_bearing, queue_size=1)
    pub = rospy.Publisher('/rover_command', RoverCommand, queue_size=1)

    rospy.spin()
