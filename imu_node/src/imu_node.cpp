#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "SparkFunLSM303C.h"
#include <sensor_msgs/Imu.h>
#include <math.h>


int main(int argc, char **argv)
{
  LSM303C chip;

  ROS_INFO("Starting node");
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;
  ros::Rate rate(100); // 10 hz

  
  chip.begin();


  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu",1000);

  sensor_msgs::Imu imu;
  
  while (ros::ok())
  {
    imu.header.stamp = ros::Time::now();
    imu.linear_acceleration.x = 0.00981*chip.readAccelX();
    imu.linear_acceleration.y = 0.00981*chip.readAccelY();
    imu.linear_acceleration.z = 0.00981*chip.readAccelZ();
    imu.orientation.x = chip.readMagX();
    imu.orientation.y = chip.readMagY();
    imu.orientation.z = chip.readMagZ();
    imu_pub.publish(imu);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
