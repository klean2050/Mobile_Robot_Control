#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
import signal
import sys
import math


# use Raspberry Pi board pin numbers
GPIO.setmode(GPIO.BCM)
# time for sensor to settle
SENSOR_SETTLE_TIME = 0.00001
# sonic speed (343 m/s)
SONIC_SPEED = 343
# list of sonars
sonars = []

def close(signal, frame):
	print "Closing sonars... \n "
	GPIO.cleanup()
	sys.exit(0)

signal.signal(signal.SIGINT, close)


def define_sonars():

	sonar_trigger = rospy.get_param("~sonar_trigger")
	sonar_echo = rospy.get_param("~sonar_echo")	
    # sonar1 with pin configuration (HEIGHT)
    	sonar1 = {'ID': 1, 'TRIG': sonar_trigger, 'ECHO': sonar_echo }
    	sonars.append(sonar1) # add to the list
    # sonar2 with pin configuration (LEFT)
    #sonar2 = {'ID': 2, 'TRIG': 27, 'ECHO': 22 }
    #sonars.append(sonar2) # add to the list

def init_sonars():

    if len(sonars) > 0:
        for sonar_current in sonars:
            
            #Sonar's trig pins should be out
            GPIO.setup( sonar_current['TRIG'], GPIO.OUT )
            GPIO.output( sonar_current['TRIG'], False)

            #Sonar's echo pins shoud be in
            GPIO.setup( sonar_current['ECHO'], GPIO.IN )

            time.sleep(0.1)


def read_sonar(sonar_id):
    
    sonar_current = sonars[sonar_id - 1]
    pinTrigger = sonar_current['TRIG']
    pinEcho = sonar_current['ECHO']
    
    # set Trigger to HIGH
    GPIO.output(pinTrigger, True)
    # set Trigger after 0.01ms to LOW
    time.sleep(SENSOR_SETTLE_TIME)
    GPIO.output(pinTrigger, False)
    #startTime = time.time()
    #stopTime = time.time()
    counter = 0
    # save start time
    while 0 == GPIO.input(pinEcho):
	counter += 1
	if counter > 10000:
		print "Jumping current measurement..."
		break
	pass
    startTime = time.time()
    counter = 0
    # save time of arrival
    while 1 == GPIO.input(pinEcho):
        pass
    stopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = stopTime - startTime
    # multiply with the sonic speed and
    # divide by 2, because there and back
    altitude = (TimeElapsed * SONIC_SPEED) / 2
    return altitude
    
def publish_sonar():
	rospy.init_node('sonar_publisher', anonymous=True)
	sonar_topic = rospy.get_param('~sonar_topic')
    	pub = rospy.Publisher(sonar_topic, Range, queue_size=1)

    	#initialize sonars
    	define_sonars()
	init_sonars()


    	#rospy.init_node('sonar_publisher', anonymous=True)
    	rate = rospy.Rate(10) # 10hz
    	current_altitude = Range()
    	while not rospy.is_shutdown():
	
		current_altitude.header.seq+= 1
        	current_altitude.header.stamp = rospy.get_rostime()
        	current_altitude.header.frame_id = "sonar_frame"
	
		current_altitude.min_range = 0.02
		current_altitude.max_range = 3.5

		sonar1 = read_sonar(1)
		#time.sleep(0.02)

		#if sonar1 >= 3.5 or sonar2 > 3.5:
			#current_altitude.range = 3.5
		current_altitude.range = sonar1	
	
        	pub.publish(current_altitude)
        	rate.sleep()
 
if __name__ == '__main__':
    try:
        publish_sonar()
    except rospy.ROSInterruptException:
    	# disable GPIO
	GPIO.cleanup()
        pass
