/*
 * Global controller to follow a trajectory
 *
 * By: Christopher Dunkers, cmdunkers@cmu.edu
 * March 1, 2015
 */


#include "trajectory_controller/global_controller.h"

/*********************************************
 *
 *
 *********************************************/
double getValueFromPolynomial(std::vector<double> alphas, double time){

  double value = 0.0;
  for(int i = 0; i < alphas.size(); i++){
    value += alphas[i] * pow(time,i);
  }
  return value;
}

/*********************************************
 *
 *
 *********************************************/
global_controller::global_controller(){
  
  //the main node handle
  ros::NodeHandle nh;

  //grab the parameters
  ros::NodeHandle private_node_handle_("~");

  //param
  //distance step for resolution

  plan_pub = nh.advertise<nav_msgs::Path>("/plan", 1000, true);
  waypoints_sub = nh.subscribe("/waypoints", 1000, &global_controller::update_waypoints, this);

  step = 0.05;

  //set up the current robot pose
  cur_robot_pose.header.stamp = ros::Time::now();
  cur_robot_pose.header.frame_id = "/world";
    
  cur_robot_pose.pose.position.x = 0.0;
  cur_robot_pose.pose.position.y = 0.0; 

  tf::Quaternion goal_quat = tf::createQuaternionFromYaw(0.0);
  cur_robot_pose.pose.orientation.x = goal_quat.x();
  cur_robot_pose.pose.orientation.y = goal_quat.y();
  cur_robot_pose.pose.orientation.z = goal_quat.z();
  cur_robot_pose.pose.orientation.w = goal_quat.w();
 
}

/*********************************************
 *
 *
 *********************************************/
void global_controller::update_waypoints(const nav_msgs::Path::ConstPtr& msg){
  bool goal_changed = false;

//  if(goal_waypoints.size() == msg->poses.size()){
//    for(int i = 0; i < msg->poses.size(); i++){
//     if(comparePoses(goal_waypoints.poses[i].pose, msg->poses[i].pose){
//        goal_changed = true;
//        break;
//      }
//    }
//  } else {
//    goal_changed = true;
//  }	

  goal_changed = true;

  //update path if goal position changed
  if(goal_changed){
    goal_waypoints.poses.clear();
    goal_waypoints = *msg;
    this->create_path();
  }
}

/*********************************************
 *
 *
 *********************************************/
void global_controller::create_path(){
  //find and store the robots current pose in the world frame (using tf)
  try{
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_link", "/world", ros::Time(0), transform);
    cur_robot_pose.pose.position.x = transform.getOrigin().x();  
    cur_robot_pose.pose.position.y = transform.getOrigin().y();
    
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(tf::getYaw(transform.getRotation()));
    cur_robot_pose.pose.orientation.x = goal_quat.x();
    cur_robot_pose.pose.orientation.y = goal_quat.y();
    cur_robot_pose.pose.orientation.z = goal_quat.z();
    cur_robot_pose.pose.orientation.w = goal_quat.w();
  } catch (tf::TransformException ex){
    ROS_WARN("%s",ex.what());
    //ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep();
  }
  
  ROS_INFO("checked for new robot position");
  
  //calc the polynomial for the two points
  // assume straight line
  // assume heading is that from the first to second point
  x_alphas.clear();
  y_alphas.clear();
  h_alphas.clear();

  double x_start = cur_robot_pose.pose.position.x;
  double y_start = cur_robot_pose.pose.position.y;

  double x_end = goal_waypoints.poses.front().pose.position.x;
  double y_end = goal_waypoints.poses.front().pose.position.y;

  ROS_INFO("x_start %f", x_start);
  ROS_INFO("x_end %f", x_end);
  ROS_INFO("y_start %f", y_start);
  ROS_INFO("y_end %f", y_end);

  std::vector<double> x_alpha_1, y_alpha_1, h_alpha_1;
  x_alpha_1.push_back(x_start);
  x_alpha_1.push_back(x_end - x_start);
  x_alphas.push_back(x_alpha_1);	  
  y_alpha_1.push_back(y_start);
  y_alpha_1.push_back(y_end - y_start);
  y_alphas.push_back(y_alpha_1);
  h_alpha_1.push_back(atan2(y_end - y_start,x_end - x_start));
  h_alphas.push_back(h_alpha_1);
  ROS_INFO("made the alphas");
  //calculate the new path
  plan.poses.clear();

  ros::Time plan_time = ros::Time::now();
  for(int i = 0; i < x_alphas.size(); i++){
    double distance = sqrt(pow(x_alphas[i].back(),2) + pow(y_alphas[i].back(),2));
    double time_increment = 1.0/double(floor(distance/this->step));
    ROS_INFO("distance %f time interval %f", distance,time_increment);
    ROS_INFO("Created the Path for first polynomial");
    
    for(double t = 0.0; t < 1.0; t += time_increment){
      geometry_msgs::PoseStamped next_pose;
      next_pose.header.stamp = plan_time;
      next_pose.header.frame_id = 
      
      next_pose.pose.position.x = getValueFromPolynomial(x_alphas[i],t);
      next_pose.pose.position.y = getValueFromPolynomial(y_alphas[i],t); 

      tf::Quaternion goal_quat = tf::createQuaternionFromYaw(getValueFromPolynomial(h_alphas[i],t));
      next_pose.pose.orientation.x = goal_quat.x();
      next_pose.pose.orientation.y = goal_quat.y();
      next_pose.pose.orientation.z = goal_quat.z();
      next_pose.pose.orientation.w = goal_quat.w();

      plan.poses.push_back(next_pose);
      //ROS_INFO("Created polynomial at time %f ", t);
    }
    //add the goal to the end of the path 
    plan.poses.push_back(goal_waypoints.poses[i]);
    ROS_INFO("Created path for polynomial %d", i);
  }
  ROS_INFO("Created the Path and Publishing");
  //publish the new path
  plan_pub.publish(plan);

}

/*********************************************
 *
 *
 *********************************************/
int main(int argc, char **argv){

  ros::init(argc, argv, "global_controller");

  global_controller gc;
  
  ROS_INFO("global path planner started!");	

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}



