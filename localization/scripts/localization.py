#!/usr/bin/env python
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Odometry
posx = 0.0
posy = 0.0
orz  = 0.0
# Sonars:
sonarF  = 0.0
sonarFL = 0.0
sonarFR = 0.0
sonarL  = 0.0
sonarR  = 0.0
# IMU:
imuRoll  = 0.0 		# orientation
imuPitch = 0.0
imuYaw   = 0.0
imuAngVelX = 0.0 	# angular velocity
imuAngVelY = 0.0
imuAngVelZ = 0.0
imuLinAccX = 0.0 	# linear acceleration
imuLinAccY = 0.0
imuLinAccZ = 0.0

x  = np.array([0.0,0.0,0.0]) 	# 0:output / 1:prediction / 2:measurement
y  = np.array([0.0,0.0,0.0])
th = np.array([12%np.pi,0.0,0.0])
vx = np.array([0.0,0.0,0.0])
ax = 0.0
wz = 0.0
dt = 0.1			# Rate = 10Hz
P = np.zeros(4)

nothing_chosen = False
boolF  = True
boolFL = True
boolFR = True
boolL  = True
boolR  = True
once   = True
x_predicted = 0
y_predicted = 0

x_odom  = []
y_odom  = []
x_pred  = []
y_pred  = []
x_error = []
y_error = []
a_error = []
time    = []
t = 0
tempx = 0.0
tempy = 0.0
tempth = 0.0

def estimate_xy():
	global sonarF,sonarL,sonarR,sonarFR,sonarFL,x,y,th,boolF,boolFL,boolFR,boolR,boolL,x_predicted,y_predicted,nothing_chosen
	limit = 1.4
	angle = 0
	sonar = 0
	d = 0
	wall_state = 0
	x0 = 0
	y0 = 0
	wall1 = 0
	wall2 = 0

	if (sonarF <= limit and boolF):
		if (0 < th[0] < np.pi/2): 	  wall_state = 12
		elif (np.pi/2 < th[0] < np.pi):   wall_state = 23
		elif (-np.pi < th[0] < -np.pi/2): wall_state = 34
		elif (-np.pi/2 < th[0] < 0): 	  wall_state = 41
		angle = th[0]
		sonar = sonarF
		boolF = False
		d = 0.05
		print("sonarF")
	elif (sonarL <= limit and boolL):
		if (0 < th[0] < np.pi/2): 	  wall_state = 23
		elif (np.pi/2 < th[0] < np.pi):   wall_state = 34
		elif (-np.pi < th[0] < -np.pi/2): wall_state = 41
		elif (-np.pi/2 < th[0] < 0):      wall_state = 12
		angle = th[0] + np.pi/2
		sonar = sonarL
		boolL = False
		d = 0.05
		print("sonarL")
	elif (sonarR <= limit and boolR):
		if (0 < th[0] < np.pi/2): 	  wall_state = 41
		elif (np.pi/2 < th[0] < np.pi):   wall_state = 12
		elif (-np.pi < th[0] < -np.pi/2): wall_state = 23
		elif (-np.pi/2 < th[0] < 0): 	  wall_state = 34
		angle = th[0] - np.pi/2
		sonar = sonarR
		boolR = False
		d = 0.05
		print("sonarR")
	elif (sonarFL <= limit and boolFL):
		if (-np.pi/3.5 < th[0] < np.pi/3.5):    		wall_state = 12
		elif (np.pi/3.5 < th[0] < 3*np.pi/3.5): 		wall_state = 23
		elif (th[0] > 3*np.pi/3.5 or th[0] < -3*np.pi/3.5):     wall_state = 34
		elif (-3*np.pi/3.5 < th[0] < -np.pi/3.5): 		wall_state = 41
		angle = th[0] + np.pi/3.5
		sonar = sonarFL
		d = np.sqrt(2) * 0.05
		boolFL = False
		print("sonarFL")
	elif (sonarFR <= limit and boolFR):
		if (-np.pi/3.5 < th[0] < np.pi/3.5): 			wall_state = 41
		elif (np.pi/3.5 < th[0] < 3*np.pi/3.5): 		wall_state = 12
		elif (th[0] > 3*np.pi/3.5 or th[0] < -3*np.pi/3.5): 	wall_state = 23
		elif (-3*np.pi/3.5 < th[0] < -np.pi/3.5): 		wall_state = 34
		angle = th[0] - np.pi/3.5
		sonar = sonarFR
		boolFR = False
		print("sonarFR")
		d = np.sqrt(2) * 0.05
	else:
		nothing_chosen = True
		return

	# determine the corner
	if (wall_state == 12):
		x0 = 0.75
		y0 = 0.75
		wall1 = 2
		wall2 = 1
	elif (wall_state == 23):
		x0 = -0.75
		y0 = 0.75
		wall1 = 2
		wall2 = 3
	elif (wall_state == 34):
		x0 = -0.75
		y0 = -0.75
		wall1 = 3
		wall2 = 4
	elif (wall_state == 41):
		x0 = 0.75
		y0 = -0.75
		wall1 = 1
		wall2 = 4

	# determine the wall
	if (y[0]+0.1*np.sin(th[0]) > np.tan(angle)*(x[0]+0.1*np.cos(th[0])-x0)+y0):
		chosen = wall1
	else:
		chosen = wall2
	# perpendicular projections
	if (chosen == 1 or chosen == 3):
		x[2] = x0 - ((sonar+d)*np.cos(angle) + 0.1*np.cos(th[0])) + x[2]
		x_predicted = x_predicted+1
		print("x",x[2])
	elif (chosen == 2 or chosen == 4):
		y[2] = y0 - ((sonar+d)*np.sin(angle) + 0.1*np.sin(th[0])) + y[2]
		y_predicted = y_predicted+1
		print("y",y[2])
	print("wall1",wall1)
	print("wall2",wall2)
	print("chosen",chosen)

def input_control():
	global imuLinAccX, imuAngVelZ, ax, wz

	if abs(imuLinAccX)<0.01: imuLinAccX = 0
	elif (imuLinAccX>2):     imuLinAccX = 2
	elif (imuLinAccX<-2):    imuLinxAccX = -2

	if abs(imuAngVelZ)<0.05: imuAngVelZ = 0
	elif (imuAngVelZ>0.25):  imuAngVelZ = 0.25
	elif (imuAngVelZ<-0.25): imuAngVelZ = -0.25

	ax = imuLinAccX
	wz = imuAngVelZ

def prediction():
	global x,y,th,vx,P,dt,ax,wz,P,posx,posy,orz

	dax = 0.002
	dwz = 0.002
	# estimated errors
	dth = dwz*dt
	dvx = dax*dt
	dx  = 1.0/2*dax*np.cos(th[0])*dt*dt
	dy  = 1.0/2*dax*np.sin(th[0])*dt*dt

	# prediction equations
	x[1]  = x[0]  + vx[0]*np.cos(th[0])*dt + 1/2*ax*np.cos(th[0])*dt*dt
	y[1]  = y[0]  + vx[0]*np.sin(th[0])*dt + 1/2*ax*np.sin(th[0])*dt*dt
	vx[1] = vx[0] +                  ax*dt
	th[1] = th[0] +			 wz*dt

	# prediction filtering
	if (x[1] > x[0]+0.02):   x[1] = x[0]+0.02
	elif (x[1] < x[0]-0.02): x[1] = x[0]-0.02
	if (y[1] > y[0]+0.02):   y[1] = y[0]+0.02
	elif (y[1] < y[0]-0.02): y[1] = y[0]-0.02
	if vx[1]>0.25:  vx[1] = 0.25
	if vx[1]<0.02: vx[1] = 0.0

	Cw = np.array([[dx**2 ,dx*dy ,0.0   ,dx*dvx],
		       [dx*dy ,dy**2 ,0.0   ,dy*dvx],
		       [0.0   ,0.0   ,dth**2,0.0   ],
	               [dx*dvx,dy*dvx,0.0   ,dvx**2]])

	Al = np.array([[1.0,0.0,-vx[0]*np.sin(th[0])*dt-1/2*ax*np.sin(th[0])*dt*dt,np.cos(th[0])*dt],
		       [0.0,1.0,+vx[0]*np.cos(th[0])*dt+1/2*ax*np.cos(th[0])*dt*dt,np.sin(th[0])*dt],
		       [0.0,0.0,1.0                                               ,0.0             ],
		       [0.0,0.0,0.0                                               ,1.0             ]])

	P = 8*(np.matmul(np.matmul(Al,P),np.transpose(Al)) + Cw)

def measurement():
	global x,y,th,vx,imuYaw,boolF,boolFL,boolL,boolR,boolFR,x_predicted,y_predicted,nothing_chosen

	boolF  = True
	boolFL = True
	boolFR = True
	boolL  = True
	boolR  = True
	nothing_chosen = False
	x_predicted = 0
	y_predicted = 0

	x[2] = 0
	y[2] = 0
	while not nothing_chosen:
		estimate_xy()

	if (x_predicted==0): x[2] = x[1]
	else: 		     x[2] = x[2]/x_predicted	# take the mean of the measurements
	if (y_predicted==0): y[2] = y[1]
	else: 		     y[2] = y[2]/y_predicted

	if (imuYaw < np.pi - 12%np.pi): th[2] = imuYaw + 12%np.pi
	else: th[2] = -2*np.pi + (imuYaw + 12 % np.pi)

	vx[2] = vx[1]
	if (abs(wz)>0.1): vx[2] = 0

	# measurement filtering
	if (x[2] > x[0]+0.1):   x[2] = x[0]+0.1
	elif (x[2] < x[0]-0.1): x[2] = x[0]-0.1
	if (y[2] > y[0]+0.1):   y[2] = y[0]+0.1
	elif (y[2] < y[0]-0.1): y[2] = y[0]-0.1

def kalman_estimation():
	global x,y,th,vx,P,R,posx,posy,orz

	R = np.array([[(0.01*np.cos(th[0]))**2,0.0                    ,0.0     ,0.0        ],
		      [0.0                    ,(0.01*np.sin(th[0]))**2,0.0     ,0.0        ],
		      [0.0                    ,0.0                    ,0.002**2,0.0        ],
	              [0.0                    ,0.0                    ,0.0     ,100*0.01**2]])

	H = np.array([[1.0,0.0,0.0,0.0],
		      [0.0,1.0,0.0,0.0],
		      [0.0,0.0,1.0,0.0],
		      [0.0,0.0,0.0,1.0]])

	temp = np.matmul(np.matmul(H,P),np.transpose(H))+R
	K = np.matmul(np.matmul(P,np.transpose(H)),inv(temp))
	P = np.matmul((np.eye(4)-np.matmul(K,H)),P)

	pred = np.array([x[1],y[1],th[1],vx[1]])
	meas = np.array([x[2],y[2],th[2],vx[2]])
	state = pred + np.matmul(K,(meas-np.matmul(H,pred)))

	x[0]  = state[0]
	y[0]  = state[1]
	th[0] = state[2]
	vx[0] = state[3]

	if vx[0] > 0.25 : vx[0] = 0.25
	if vx[0] < 0.02 : vx[0] = 0.0

	print "Estimated x:        " + str(x[0])
	print "Estimated y:        " + str(y[0])
	print "Estimated theta:    " + str(th[0])
	print "EStimated velocity: " + str(vx[0])
	print "Time:               " + str(t/10.0) +" sec"
	print ""

def error_diagrams():
	global x_error,y_error,a_error,once,t,posx,posy,orz,th,x,y,time,x_odom,y_odom,x_pred,y_pred,tempx,tempy,tempth
	t = t + 1
	time.append(0.1*t)

	tempx = abs(posx-x[0])+tempx
	tempy = abs(posy-y[0])+tempy
	tempth = abs(orz-th[0])+tempth

	x_error.append(posx-x[0])
	y_error.append(posy-y[0])
	a_error.append(orz-th[0])

	x_odom.append(posx)
	y_odom.append(posy)
	x_pred.append(x[0])
	y_pred.append(y[0])

	# when done, plot
	if (t == 100 and once):
		once = False
		plt.figure(1)

		print "EKF Localization Process finished!"
		print ""
		print "Mean error for x:  " + str(tempx/1800.0)
		print "Mean error for y:  " + str(tempy/1800.0)
		print "Mean error for th: " + str(tempth/1800.0)

		plt.subplot(311)
		plt.plot(time,x_error)
		plt.xlabel('Time (sec)')
		plt.ylabel('Error (m)')
		plt.title('x error')

		plt.subplot(312)
		plt.plot(time,y_error)
		plt.xlabel('Time (sec)')
		plt.ylabel('Error (m)')
		plt.title('y error')

		plt.subplot(313)
		plt.plot(time,a_error)
		plt.xlabel('Time (sec)')
		plt.ylabel('Error (rad)')
		plt.title('th error')

		plt.figure(2)
		plt.scatter(x_odom,y_odom,c="b")
		plt.scatter(x_pred,y_pred,c="r")
		plt.legend(loc="upper left")

		plt.show()


def send_velocity():
	global x,y,th,vx,wz

    	input_control();
    	prediction();
	measurement();
	kalman_estimation();
	error_diagrams();

#def odomCallback(msg):
#	global posx,posy,orz
#    	posx = msg.pose.pose.position.x
#	posy = msg.pose.pose.position.y
#
#	orient = msg.pose.pose.orientation
#    	orient_list = [orient.x, orient.y, orient.z, orient.w]
#    	(odomRoll, odomPitch, odomYaw) = euler_from_quaternion(orient_list)
#	orz  = odomYaw

def sonarFrontCallback(msg):
	global sonarF
    	sonarF = msg.range;
    	send_velocity()

def sonarFrontLeftCallback(msg):
	global sonarFL
    	sonarFL = msg.range

def sonarFrontRightCallback(msg):
	global sonarFR
    	sonarFR = msg.range

def sonarLeftCallback(msg):
	global sonarL
   	sonarL = msg.range

def sonarRightCallback(msg):
	global sonarR
    	sonarR = msg.range

def imuCallback(msg):
    	global imuYaw,imuAngVelZ,imuLinAccX
    	# orientation: quaternion to RPY
    	orientation_q = msg.orientation
    	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    	(imuRoll, imuPitch, imuYaw) = euler_from_quaternion (orientation_list)
	imuYaw = -imuYaw
    	# angular velocity
    	imuAngVelZ = msg.angular_velocity.z
    	# linear acceleration
    	imuLinAccX = msg.linear_acceleration.x

def localizer_py():
    	rospy.init_node('localizer_node', anonymous=True)
	#rospy.Subscriber("odom", Odometry, odomCallback)
    	rospy.Subscriber("sonar_front", Range, sonarFrontCallback)
   	rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    	rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    	rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    	rospy.Subscriber("sonar_right", Range, sonarRightCallback)
    	rospy.Subscriber("imu/data", Imu, imuCallback)
	rate = rospy.Rate(10)
    	while not rospy.is_shutdown():
        	rospy.spin()

if __name__ == '__main__':
    	try:
        	localizer_py()
    	except rospy.ROSInterruptException: pass
