/*
 * Perception tracker finds robot and sends pose in world
 *
 * By: Eric Danziger
 * March 1, 2015
 */

#ifndef PERCEPTION_TRACKER_H_
#define PERCEPTION_TRACKER_H_

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/LinearMath/Transform.h"
#include "perception_system/bot.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

//#include "oddbot_msgs/MotorCommand.h"
#include <cmath> 



class perceptionTracker
{
public:
perceptionTracker();
~perceptionTracker();
void publishMessage(ros::Publisher *pub_message);
geometry_msgs::Pose2D robotPose;
geometry_msgs::Pose2D goalPose;


private:
nav_msgs::Path currentPath;
int pathCounter;
geometry_msgs::Twist currentVel;
geometry_msgs::Pose currentPose;

};


#endif
