#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <math.h>

#include <sstream>

#define M_PI  3.14159265358979323846
using namespace std;

float sonarF_val, sonarFL_val, sonarFR_val, sonarL_val, sonarR_val;
double roll, pitch, yaw;
double d = 0.05; /* distance between consecutive sonars */

void sonarFrontCallback(const sensor_msgs::Range& msg){

   sonarF_val = msg.range;
   ROS_INFO_STREAM("Front Sonar's indication: "<<sonarF_val);
}

void sonarFrontLeftCallback(const sensor_msgs::Range& msg){

   sonarFL_val = msg.range;
   //ROS_INFO_STREAM("Front-Left Sonar's indication: "<<sonarFL_val);
}

void sonarFrontRightCallback(const sensor_msgs::Range& msg){

   sonarFR_val = msg.range;
   //ROS_INFO_STREAM("Front-Right Sonar's indication: "<<sonarFR_val);
}

void sonarLeftCallback(const sensor_msgs::Range& msg){

   sonarL_val = msg.range;
   //ROS_INFO_STREAM("Left Sonar's indication: "<<sonarL_val);
}

void sonarRightCallback(const sensor_msgs::Range& msg){

   sonarR_val = msg.range;
   //ROS_INFO_STREAM("Right Sonar's indication: "<<sonarR_val);
}

void imuCallback(const sensor_msgs::Imu& msg){

  tf::Quaternion q(
  msg.orientation.x,
  msg.orientation.y,
  msg.orientation.z,
  msg.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

double deg2rad(const int deg){

  double rad = deg * (M_PI/180);

  return rad;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "walker");
  ros::NodeHandle n;
  ros::Subscriber sonarFront_sub = n.subscribe("sonar_front", 1, &sonarFrontCallback);
  ros::Subscriber sonarFrontLeft_sub = n.subscribe("sonar_front_left", 1, &sonarFrontLeftCallback);
  ros::Subscriber sonarFrontRight_sub = n.subscribe("sonar_front_right", 1, &sonarFrontRightCallback);
  ros::Subscriber sonarLeft_sub = n.subscribe("sonar_left", 1, &sonarLeftCallback);
  ros::Subscriber sonarRight_sub = n.subscribe("sonar_right", 1, &sonarRightCallback);
  ros::Subscriber imu_sub = n.subscribe("imu/data", 1, &imuCallback);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  int rosFreqHz = 10;
  ros::Rate loop_rate(rosFreqHz);

  /** PHASE EXPLANATION
  1. move forward
  2. rotate cw
  3. rotate ccw
  */
  int phase = 1;
  float linX_move = 0.2;
  float angZ_move = 1.5;
  float linX = 0.0;
  float angZ = 0.0;
  double fsfw = 0.2; // front sonar's threshold under which the linear motion is stopped (3.5cm from front sonar, but 3cm from chassis)
  double flsfw = 0.1; // front-left sonar's threshold under which the linear motion is stopped (2.5cm from front sonar, but 2cm from chassis)
  double frsfw = 0.1; // front-right sonar's threshold under which the linear motion is stopped (2.5cm from front sonar, but 2cm from chassis)
  double counter = 0;
  double timeTarget = 0;
  double counterTotal = 0;
  double timeTotalSim = 3*60;

  sonarF_val = 1000;
  sonarFL_val = 1000;
  sonarFR_val = 1000;
  sonarL_val = 1000;
  sonarR_val = 1000;

  /* initialize random seed: */
  srand (time(NULL));

  while ( (ros::ok()) && (counterTotal <= timeTotalSim) )
  {

    counterTotal += 1 /(double)rosFreqHz;

    geometry_msgs::Twist velocity;

    if (phase==1) {

      if ( (sonarF_val < fsfw) || (sonarFL_val < flsfw) || (sonarFR_val < frsfw) ) {

        linX = 0.0;
        angZ = 0.0;

        double rand_val = M_PI/2 + deg2rad(rand()%40 - 20);
        counter = 0;
        timeTarget = rand_val*2 / angZ_move;

        if (sonarFL_val < sonarFR_val)
          phase = 2;
        else if (sonarFL_val >= sonarFR_val)
          phase = 3;
      }
      else {

        linX = linX_move;
        angZ = 0.0;
      }
    }

    else if (phase==2) {

      counter += 1 /(double)rosFreqHz;

      if (counter > timeTarget){

        linX = 0.0;
        angZ = 0.0;
        phase = 1;
      }
      else {

	cout << "Turninggg" << endl;
        linX = 0.0;
        angZ = angZ_move;
      }

    }

    else if (phase==3) {

      counter += 1 /(double)rosFreqHz;

      if (counter > timeTarget){

        linX = 0.0;
        angZ = 0.0;
        phase = 1;
      }
      else {

        linX = 0.0;
        angZ = -angZ_move;
      }

    }
    
    ROS_INFO_STREAM("Phase: "<<phase);
    ROS_INFO_STREAM("Linear Velocity: "<<linX);
    ROS_INFO_STREAM("Angular Velocity: "<<angZ);

    velocity.linear.x = linX;
    velocity.angular.z = angZ;
    velocity_pub.publish(velocity);

    loop_rate.sleep();
    ros::spinOnce();
  }


  return 0;
}
