#!/usr/bin/env python
import time
import RPi.GPIO as GPIO

class HCSR04:
    """
    Driver to use the untrasonic sensor HC-SR04.
    The sensor range is between 2cm and 4m.
    """

    def __init__(self, trigger_pin, echo_pin, distance_calibration=0.5):
        """
        trigger_pin: Output pin to send pulses
        echo_pin: Readonly pin to measure the distance. The pin should be protected with 1k resistor
        By default is based in sensor limit range (4m)

        """
        #print("Init HC-SR04 Library")
        
        # GPIO Clean-up is usually not in the beginning, 
        # but it solves crash related restart issues
        try:
            GPIO.setwarnings(False)
            GPIO.cleanup()
            GPIO.setwarnings(True)
            #print("HC-SR04: GPIO Cleanup")
        except:
            pass

        GPIO.setmode(GPIO.BCM)
            
        # Init trigger pin (out)
        self.trigger = trigger_pin
        GPIO.setup(self.trigger,GPIO.OUT)

        # Init echo pin (in)
        self.echo = echo_pin
        GPIO.setup(self.echo,GPIO.IN)
        
        self.distance_calibration = distance_calibration
    
    def __del__(self):
        """Once finished using the sensor, switch to standby mode."""
        self.shutdown()
        
    def shutdown(self):
        #print("Shutdown HC-SR04 Library")
        GPIO.setwarnings(False)
        GPIO.cleanup()
    
    def get_distance_cm(self):
        """
        Get the distance in centimeters with rounded to 2 decimals.
        It returns a float or -1 if out of range
        """
        
        # Define blank distance
        distance = -1
        pulse_duration = 0
        
        # Check if echo pin is HIGH, if yes, drop this get_function_cm
        # Failsafe in case a second process is interacting
        if GPIO.input(self.echo) == 1:
            return distance
        
        GPIO.output(self.trigger, False) #Set TRIG as LOW
        # Waitng For Sensor To Settle
        time.sleep(0.000005)

        GPIO.output(self.trigger, True)#Set TRIG as HIGH
        time.sleep(0.00001)
        GPIO.output(self.trigger, False) #Set TRIG as LOW
        
        # Using start and duration timeouts to prevent CPU runaway
        # and process lockdown in case while condition cannot be met
        timeout_start = time.time()
        #Check whether the ECHO is LOW
        while GPIO.input(self.echo)==0 and (time.time()-timeout_start) < 0.03: 
            #Saves the last known time of LOW pulse
            pulse_start = time.time()
        
        while GPIO.input(self.echo)==1 and pulse_duration < 0.03: #Check whether the ECHO is HIGH
            #Saves the last known time of HIGH pulse
            pulse_end = time.time() 
            
            # In case ECHO LOW was not registered it would throw an error here
            try:
                #Get pulse duration to a variable
                pulse_duration = pulse_end - pulse_start 
            except:
                pulse_duration = 0
            
            #Multiply pulse duration by 17150 to get distance
            distance = pulse_duration * 17150
            
            #Check whether the distance is within range
            if distance > 2 and distance < 400:
                distance = distance - self.distance_calibration
            else:
                # Distance is -1 for out of range status
                distance = -1
        
        return distance

