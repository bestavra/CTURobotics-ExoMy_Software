#!/usr/bin/env python
from sensor_msgs.msg import Temperature,RelativeHumidity
import rospy
import RPi.GPIO as GPIO
import dht11

def env_readout():
    env_pub_humidity = rospy.Publisher("/relative_humidity", RelativeHumidity, queue_size=1)
    env_pub_temp = rospy.Publisher("/temperature", Temperature, queue_size=1)
    
    rospy.loginfo("DHT11 callback!")
    
    env_temp = Temperature()
    env_humidity = RelativeHumidity()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.cleanup()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        instance = dht11.DHT11(pin = 4)
        result = instance.read()

        if result.is_valid():
            env_temp.temperature = result.temperature
            env_humidity.relative_humidity = result.humidity
            env_pub_temp.publish(env_temp)        
            env_pub_humidity.publish(env_humidity)
        else:
            pass

        rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('dht11_node')
    rospy.loginfo("Starting the DHT11 node")
    try:
        env_readout()
    except rospy.ROSInterruptException:
        pass