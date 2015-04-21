/*
 * Local controller to follow a trajectory
 *
 * By: Eric Danziger
 * March 1, 2015
 */

#ifndef LOCAL_CONTROLLER_H_
#define LOCAL_CONTROLLER_H_

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/LinearMath/Transform.h"

#include "oddbot_msgs/MotorCommand.h"
#include <cmath> 



class localController
{
 public:
  localController();
  ~localController();
  void publishMessage(ros::Publisher *pub_message);
  void pathMessageCallback(const nav_msgs::Path::ConstPtr &msg);
  void odomUpdateCallback(const nav_msgs::Odometry::ConstPtr &msg);
 private:
  nav_msgs::Path currentPath;
  int pathCounter;
  geometry_msgs::Twist currentVel;
  geometry_msgs::Pose currentPose;
};


#endif
