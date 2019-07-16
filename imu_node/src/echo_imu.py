#!/usr/bin/env python
import rospy
import numpy as np
import sys
from sensor_msgs.msg import Imu
from math import asin,cos,sin,sqrt,atan2,pi

linear_acc = np.array([0.0, 0.0, 0.0])
magnetometer = np.array([0.0, 0.0, 0.0])
velocity = np.array([0.0,0.0,0.0])
state = np.array([0.0,0.0,0.0,0.0])
lasttime = 0
offset = np.array([0.0, 0.0, 0.0])
offset_angle = 0.0
offset_flag = 1

def callback(msg):
    global linear_acc
    global magnetometer
    global velocity
    global state
    global lasttime
    global offset
    global offset_angle
    global offset_flag

    if offset_flag:
        offset_flag = 0
        lasttime = msg.header.stamp
        magnetometer = np.array([msg.orientation.y,-msg.orientation.z, msg.orientation.x])
        magnetometer = magnetometer/sqrt(magnetometer[0]**2 + magnetometer[1]**2 + magnetometer[2]**2)
        offset_angle = atan2(magnetometer[1],magnetometer[0])
        offset = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y, msg.linear_acceleration.z])
        #print offset
        return

    filter_coef = 0.8
    magnetometer = filter_coef*magnetometer + (1-filter_coef)*np.array([msg.orientation.y,-msg.orientation.z, msg.orientation.x])
    magnetometer = magnetometer/sqrt(magnetometer[0]**2 + magnetometer[1]**2 + magnetometer[2]**2)
    state[3] =  offset_angle - atan2(magnetometer[1],magnetometer[0])
    if state[3] > pi:
        state[3] = state[3] - 2*pi
    elif state[3] <= -pi:
        state[3] = state[3] + 2*pi
        
    new_offset_x = cos(state[3])*offset[0] + sin(state[3])*offset[1]
    new_offset_y = -sin(state[3])*offset[0] + cos(state[3])*offset[1]
    new_offset = np.array([new_offset_x, new_offset_y, offset[2]])
    linear_acc = filter_coef*linear_acc + (1-filter_coef)*(np.array([msg.linear_acceleration.x,msg.linear_acceleration.y, msg.linear_acceleration.z])-new_offset)
    #Axn = linear_acc[0]/sqrt(linear_acc[0]**2 + linear_acc[1]**2 + linear_acc[2]**2)
    #Ayn = linear_acc[1]/sqrt(linear_acc[0]**2 + linear_acc[1]**2 + linear_acc[2]**2)
    #Pitch = asin(-Axn)
    #Roll = asin(Ayn/cos(Pitch))  
    #Mxh = magnetometer[0]*cos(Pitch)+ magnetometer[2]*sin(Pitch)
    #Myh = magnetometer[0] *sin(Roll)*sin(Pitch)+magnetometer[1]*cos(Roll)-magnetometer[2]*sin(Roll)*cos(Pitch)
    #Mzh = -magnetometer[0] *cos(Roll)*sin(Pitch)+magnetometer[1]*sin(Roll)+magnetometer[2]*cos(Roll)*cos(Pitch)
    #Heading = 180*atan2(1000*Myh,1000*Mxh)/pi
    #check = sqrt(Mxh**2 + Myh**2 + Mzh**2)
    dt = (msg.header.stamp -lasttime).to_sec()
    lasttime = msg.header.stamp
    state[:3] = state[:3] + velocity*dt + 0.5*linear_acc*dt**2
    velocity = velocity + linear_acc*dt
    #print 180/pi*state[3]
    #print dt
    #print offset_angle
    #print "Heading: {0}".format(180/pi*atan2(magnetometer[1],magnetometer[0]))
    #print "Pitch: {0} Roll: {1} Heading: {2} Check: {3}".format(Pitch,Roll,Heading,check)
    #print "Pitch: {0} Roll: {1} Heading: {2} Check: {3}".format(msg.orientation.x, msg.orientation.y, msg.orientation.z ,check)


def main(args):
    global state
    global linear_acc
    global velocity
    rospy.init_node('imu_subscriber')
    rospy.Subscriber("imu",Imu,callback)
    #rospy.spin()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #print velocity
        #print state
        # print linear_acc
        print state[3]*180/pi
        rate.sleep()


if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: 
        pass
