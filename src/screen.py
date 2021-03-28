#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
import numpy as np
import rospkg

#Exomy ST7735s library
from lib.py_st7735s import ST7735s

#Image support
from PIL import Image, ImageDraw, ImageFont

#Text support
from textwrap import fill

#Video support
#from contextlib import closing
#from videosequence import VideoSequence

#Init Package Path
rospack = rospkg.RosPack()
file_path = rospack.get_path('exomy')

#Init screen
class ExoMyScreen:
    '''
    Screen class contains all functions to control the ExoMy Screen
    '''
    
    def __init__(self):
        
        self.width=160
        self.height=128
        
        startup_message = "Hallo!\nIch bin\nRobbi!"
        
        self.screen = ST7735s(
                    port=0,
                    cs=0,  
                    dc=22,
                    rst=18,
                    backlight=16,               
                    rotation=0,
                    spi_speed_hz=16000000,
                    width=self.width,
                    height=self.height,
                    invert=False,
                    offset_left=0,
                    offset_top=0
        )
        
        self.talk(startup_message, fontsize=19)

    def display_image(self, imagepath):
        #image = Image.new("RGB", (self.width, self.height))
        try:
            image = Image.open(imagepath)
            image = image.resize((self.width, self.height))
            self.screen.display(image)
        except:
            self.talk("Error!")
    
    '''
    # not working: videosequence requires GST libraries not installed
    def display_video(self, videopath, start_frame=0, end_frame=None):
        try:
            with closing(VideoSequence(videopath)) as frames:
                #for idx, frame in enumerate(frames[start_frame:end_frame]):
                for idx, frame in enumerate(frames[100:]):
                    frame = frame.resize((self.width, self.height))
                    self.screen.display(frame)
        except:
            self.talk("Error!")
    '''
    def talk(self, message, fontsize=15):
        message=str(message)
        speach_bubble_image = Image.open(file_path + "/resources/talk_mouth.png")
        text_box_top = 28
        text_box_left = 28
        text_box_width = 104
        text_box_height = 72
        max_lines = 4
        
        fnt_path = file_path + "/resources/fonts/Orkney/Orkney Regular.ttf"
        
        if message == "":
            message = "..."
        
        # If not explictily set, the text will recieve line breaks
        if message.count("\n") == 0:
            # longer messages get broken later to allow for more text
            if len(message) > 56:
                message=str(fill(message, 14))
            else:
                message=str(fill(message, 10))
        
        # Count lines and shorten message to max_lines
        message = message.splitlines(True)
        if len(message) > max_lines:
            del message[max_lines:]
            message.append("...")
        
        output_message_text = ""
        
        output_message = output_message_text.join(message)
       
        d = ImageDraw.Draw(speach_bubble_image)
        width_text = 0
        height_text = 0
        
        while width_text < text_box_width and height_text < text_box_height:
            # iterate until the text size is just larger than the criteria
            fontsize += 1
            fnt = ImageFont.truetype(fnt_path, fontsize)
            width_text, height_text = d.textsize(output_message, fnt)
        
        ''' 
        rospy.loginfo("fontsize: " + str(fontsize))
        rospy.loginfo("width_text: " + str(width_text))
        rospy.loginfo("height_text: " + str(height_text))
        rospy.loginfo("output_message: " + str(output_message))
        '''
        
        text_box_left_centered = text_box_left + (text_box_width / 2 - width_text / 2)
        text_box_top_centered = text_box_top + (text_box_height / 2 - height_text / 2)
        
        d.multiline_text((text_box_left_centered,text_box_top_centered), output_message, font=fnt, fill=(0,0,0), align="center")
        self.screen.display(speach_bubble_image)
    
    def backlight(self, state):
        if state == True or state == False:
            self.screen.set_backlight(state)
    
    def shutdown(self):
        self.talk("Shutting down...\nBye Bye!")
    
    def close():
        self.screen.close()
    
    