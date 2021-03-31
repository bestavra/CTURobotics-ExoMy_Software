#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
import lib.py_hcsr04 as py_hcsr04
import message_filters

global ranger
ranger = py_hcsr04.HCSR04(trigger_pin=27, echo_pin=22, distance_calibration=3)

def ultrasonic_range():
    global ranger
    pub = rospy.Publisher('range', Range, queue_size=1)
    # 5Hz / Sensor could support 10Hz but the process
    # would use a lot of CPU in bad cases
    rate = rospy.Rate(10)
    
    range = Range()
    range.radiation_type = 0
    range.min_range = 0.02
    range.max_range = 4.00
    
    while not rospy.is_shutdown():
        range.range = ranger.get_distance_cm()/100
        pub.publish(range)
        rate.sleep()
        
    rospy.on_shutdown(shutdown_ranger)
    
def shutdown_ranger():
    global ranger
    ranger.shutdown()
    
if __name__ == '__main__':
    rospy.init_node('range', anonymous=True)
    rospy.loginfo("Starting the range node")
    try:
        ultrasonic_range()
    except rospy.ROSInterruptException:
        pass
