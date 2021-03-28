#!/usr/bin/env python
import rospy
import os
import time
from exomy.msg import Screen
from screen import ExoMyScreen
import rospkg

#Init Package Path
rospack = rospkg.RosPack()
file_path = rospack.get_path('exomy')

screen = ExoMyScreen()

def callback(message):
    if message.state is not "":
        screen_mood(message.state)
    elif message.screen_message is not "":
        screen_talk(message.screen_message)
    else:
        screen.display_image("Error")
    
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
    elif (message == "kisss"):
        image_path = file_path + "/resources/kiss_mouth.png"

    try:
        rospy.loginfo("Showing: " + str(image_path))
        screen.display_image(image_path)
    except:
        screen.talk("Error!")
    
def screen_talk(message):
    screen.talk(message)
    rospy.loginfo("Displaying text: " + str(message))

if __name__ == '__main__':
    rospy.init_node('screen')
    rospy.loginfo("Starting the screen node")

    try:
        screen_state = rospy.Subscriber("/state", Screen, callback, queue_size=1)
        screen_message = rospy.Subscriber("/screen_message", Screen, callback, queue_size=1)

    except rospy.ROSInterruptException:
        screen.close()
        pass
        
    rate = rospy.Rate(10)
    rospy.spin()
    