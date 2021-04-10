#!/usr/bin/env python
import rospy
import os
import time
from exomy.msg import Screen
from screen import ExoMyScreen
import rospkg

# Import MotorCommands to monitor the Commands, display status
# and trigger watchdog
from exomy.msg import MotorCommands

global motor_messages

#Init Package Path
rospack = rospkg.RosPack()
file_path = rospack.get_path('exomy')

screen = ExoMyScreen()

global watchdog_timer

def callback(message):
    if message.state is not "":
        screen_mood(message.state)
    else:
    #elif message.screen_message is not "":
        screen_talk(message.screen_message)
    #else:
    #    screen.display_image("Error")
    global watchdog_timer
    watchdog_timer.shutdown()
    watchdog_timer = rospy.Timer(rospy.Duration(60.0), watchdog, oneshot=True)
    
def screen_mood(message):
    global file_path
    
    if (message == "happy"):
        image_path = file_path + "/resources/happy_mouth.png"
    elif (message == "surprised"):
        image_path = file_path + "/resources/surprised_mouth.png"
    elif (message == "closed"):
        image_path = file_path + "/resources/closed_mouth.png"
    elif (message == "half_closed"):
        image_path = file_path + "/resources/half_closed_mouth.png"
    elif (message == "kiss"):
        image_path = file_path + "/resources/kiss_mouth.png"
    elif (message == "tongue"):
        image_path = file_path + "/resources/tongue_mouth.png"    

    try:
        rospy.loginfo("Showing: " + str(image_path))
        screen.display_image(image_path)
    except:
        screen.talk("Error!")
    
def screen_talk(message):
    screen.talk(message)
    rospy.loginfo("Displaying text: " + str(message))

#Motor watchdog monitors the motor commands and if no movement was done, status is set
def motor_watchdog(message):
    global watchdog_timer
    watchdog_timer.shutdown()
    #watchdog_timer = rospy.Timer(rospy.Duration(10.0), watchdog, oneshot=True)

def watchdog(event):
    rospy.loginfo("Status watchdog commanded...")
    screen.status()

def shutdown():
    screen.shutdown()
    time.sleep(5)
    screen_mood("closed")
    screen.close()

if __name__ == '__main__':
    rospy.init_node('screen')
    rospy.loginfo("Starting the screen node")

    try:
        screen_state = rospy.Subscriber("/state", Screen, callback, queue_size=1)
        screen_message = rospy.Subscriber("/screen_message", Screen, callback, queue_size=1)
        # Motor messages for status message support
        motor_message = rospy.Subscriber("/motor_commands", MotorCommands, motor_watchdog, queue_size=1)

    except rospy.ROSInterruptException:
        screen.close()
        pass
    
    watchdog_timer = rospy.Timer(rospy.Duration(60.0), watchdog, oneshot=True)
    
    rospy.on_shutdown(shutdown)
    
    rate = rospy.Rate(10)
    rospy.spin()
    