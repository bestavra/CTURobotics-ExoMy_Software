#!/usr/bin/env python
import rospy
from exomy.msg import Compass
import lib.py_hmc5883l as py_hmc5883l

def compass_bearing():
    pub = rospy.Publisher('bearing', Compass, queue_size=1)
    rospy.init_node('compass', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    gauss_set = rospy.get_param("gauss")
    declination_set = rospy.get_param("declination")
    calibration_set = rospy.get_param("calibration")
    compass = py_hmc5883l.HMC5883L(gauss = gauss_set, declination = declination_set)
    compass.set_calibration(calibration_set)
    rospy.loginfo("Compass started")
    rospy.loginfo("Compass declination: " + str(compass.get_declination()))
    rospy.loginfo("Compass calibration: " + str(compass.get_calibration()))
    
    while not rospy.is_shutdown():
        bearing = round(compass.get_bearing(),0)
        pub.publish(bearing)
        rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("Starting the compass node")
    try:
        compass_bearing()
    except rospy.ROSInterruptException:
        pass
