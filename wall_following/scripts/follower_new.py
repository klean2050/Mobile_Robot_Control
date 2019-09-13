#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import numpy as np
import math

maxz = 0.2
target = 0.15
target1 = 0.175
lim = 0.2
margin = 0.0

init = 2

sonarL = init
sonarF = init
sonarFL = init
sonarFR = init
sonarR = init
sonarL_prev = init
sonarF_prev = init
sonarFL_prev = init
sonarFR_prev = init
sonarR_prev = init
state = 0
counter = 0
t = 0
flag = True

def get_vel():
        global state,sonarFL,sonarL,sonarL_prev,target,target1,maxz,sonarF,sonarR,sonarFR,sonarFL_prev,sonarFR_prev,sonarF_prev,sonarR_prev
        vel = Twist()

        if state == 0:
                # FIND WALL
                vel.linear.x = 0.08
                vel.angular.z = 0.08
        elif state == 1:
                # FOUND WALL - TURN
                #if (sonarF > 0.35): sonarF = sonarF_prev
                vel.linear.x = 0.08
                vel.angular.z = -0.8
        elif state == 2:
                # ALIGN - FOLLOW WALL
                #if (sonarFL > 0.33): sonarFL = sonarFL_prev
		#if (sonarL > 0.2): sonarL = sonarL_prev
                Kp = 2
                Kd = 20
                error  = (target1-sonarFL) + (target-sonarL)
                #error = target1 - sonarFL + target - sonarL
		#error = target-sonarL
		#errord = sonarL_prev-sonarL
		errord = sonarL_prev - sonarL + sonarFL_prev - sonarFL
                vel.angular.z = -max(min(Kp*error + Kd*errord/0.1,maxz),-maxz)
                vel.linear.x = 0.08
        else :
                # STOP
                vel.linear.x = 0.0
                vel.angular.z = 0.0

        sonarL_prev = sonarL
	sonarFL_prev = sonarFL
	sonarFR_prev = sonarFR
	sonarF_prev = sonarF
	sonarR_prev = sonarR
        return vel


def define_state():
        global state,sonarF,sonarFL,sonarFR,sonarL,flag,counter,margin

        #if (counter == 5 and sonarF < 0.8):
                #state = 3
        if min(sonarF,sonarFL,sonarFR) >= 0.25  and flag :
                state = 0
                print("i am state 0")
        else:
                flag = False
                if (sonarL<sonarF + margin and  sonarFL<sonarF + margin):
                        state = 2
                else:
                        state = 1

def send_velocity():
        velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        velocity = Twist()
        define_state()
        velocity = get_vel()
        velocity_pub.publish(velocity)
        print "sonarFront: " + str(sonarF)
        print "sonarFrontLeft: " + str(sonarFL)
        print "sonarLeft: " + str(sonarL)
        print "state: " + str(state)
        print ""

def sonarFrontCallback(msg):
	global sonarF,sonarF_prev,lim
#    if (msg.range > sonarF + lim):
#	sonarF = sonarF_prev
#    elif (msg.range < sonarF - lim):
#	sonarF = sonarF_prev
#    else:
	sonarF = msg.range
        if (sonarF >0.8): sonarF = sonarF_prev
        send_velocity()

def sonarFrontLeftCallback(msg):
    	global sonarFL,sonarFL_prev,lim
#    if (msg.range > sonarFL + lim):
#	sonarFL = sonarFL_prev
#    elif (msg.range < sonarFL - lim):
#	sonarFL = sonarFL_prev
#    else:
        sonarFL = msg.range
	if (sonarFL>0.8): sonarFL = sonarFL_prev
	if (sonarFL>0.33) and (state==2): sonarFL =sonarFL_prev
def sonarFrontRightCallback(msg):
   	 global sonarFR,sonarFR_prev,lim
#    if (msg.range > sonarFR + lim):
#	sonarFR = sonarFR_prev
#    elif (msg.range < sonarFR - lim):
#	sonarFR = sonarFR_prev
#    else:
       	 sonarFR = msg.range
	 if (sonarFR > 0.8): sonarFR = sonarFR_prev

def sonarLeftCallback(msg):
    	global sonarL,sonarL_prev,lim
#    if (msg.range > sonarL + lim):
#	sonarL = sonarL_prev
#    elif (msg.range < sonarL - lim):
#	sonarL = sonarL_prev
#    else:
    	sonarL = msg.range
	if (sonarL >0.8): sonarL = sonarL_prev
	if (sonarL >0.2): sonarL = sonarL_prev
def sonarRightCallback(msg):
    	global sonarR,sonarR_prev,lim
 #   if (msg.range > sonarR + lim):
#	sonarR = sonarR_prev
#    elif (msg.range < sonarR - lim):
#	sonarR = sonarR_prev
#    else:
    	sonarR = msg.range
	if (sonarR > 0.8): sonarR = sonarR_prev

def follower_py():
    rospy.init_node("follower_node", anonymous=True)
    rospy.Subscriber("sonar_front", Range, sonarFrontCallback)
    rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    rospy.Subscriber("sonar_right", Range, sonarRightCallback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException: pass
