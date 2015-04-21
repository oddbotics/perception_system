/*
 * Global controller to follow a trajectory
 *
 * By: Christopher Dunkers, cmdunkers@cmu.edu
 * March 1, 2015
 */

#ifndef GLOBAL_CONTROLLER_H_
#define GLOBAL_CONTROLLER_H_

#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <string>
#include <cmath>

class global_controller {

  public:
    global_controller();
 
  private:
    void create_path();
    void update_waypoints(const nav_msgs::Path::ConstPtr& msg);
    nav_msgs::Path goal_waypoints;
    tf::TransformListener tf_listener;
    
    double step;
    geometry_msgs::PoseStamped cur_robot_pose;
    nav_msgs::Path plan;

    //alphas start with the constant term and increase 
    //ie a[0] + a[1]*t + a[2]*t^2 ...
    std::vector<std::vector<double> > x_alphas;
    std::vector<std::vector<double> > y_alphas;
    std::vector<std::vector<double> > h_alphas;

    ros::Subscriber waypoints_sub;
    ros::Publisher plan_pub;

};


#endif
