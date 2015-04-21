/*
 * Peception tracker for Robot
 *
 * By: Eric Danziger
 * March 1, 2015
 */


#include "perception_system/perception_tracker.h"


perceptionTracker::perceptionTracker()
{
  currentPose.position.x = 0;
  currentPose.position.y = 0;
  
}

perceptionTracker::~perceptionTracker()
{
}

void perceptionTracker::publishMessage(ros::Publisher *pub_message)
{
  //Go through the path, find all poses within 1m of currentPose
  nav_msgs::Path pathWindow;
  double timeStep = .2;
  bool alreadyLast = false;
  std::vector<double> dVel,dYaw;
  double closestDist = 10000;
  int closestIndex;
  geometry_msgs::Twist atDes, toDes;
  geometry_msgs::Twist msg;
 
  pub_message->publish(msg);

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_tracker");
  ros::NodeHandle nh;
  Bot oddBot();
  float xRobot,yRobot,tRobot;
  perceptionTracker *perc_track= new perceptionTracker();

  ros::Publisher rPub = nh.advertise<geometry_msgs::Pose2D>("/robot_perceived_pose",10);
  ros::Publisher gPub = nh.advertise<geometry_msgs::Pose2D>("/goal_perceived_pose",1,true);

  perc_track->goalPose.x = 1.9;
  perc_track->goalPose.y = 2.5;
  gPub.publish(perc_track->goalPose);


  ros::Rate r(30);

  cv::VideoCapture capture1(1); // open the video file for reading
  cv::VideoCapture capture2(2); 
  //VideoCapture capture1(1); // open the video file for reading
  //capture0.set(CV_CAP_PROP_FRAME_WIDTH,640);
  //capture0.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  capture1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  capture1.set(CV_CAP_PROP_FRAME_HEIGHT,720);
  capture2.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  capture2.set(CV_CAP_PROP_FRAME_HEIGHT,720);
     

  if(!capture1.isOpened()){
    cout << "Failed to connect to the camera 1." << endl;
  }
  if(!capture2.isOpened()){
    cout << "Failed to connect to the camera 2." << endl;
  }    


  while(nh.ok())
    {
      //Do the OPENCV stuff

      Mat imgL,imgR;
      capture1>>imgL;
      capture2>>imgR;
      if(imgL.empty()){
	cout << "Failed to capture an image on camera 1" << endl;
	return -1;
      }
      if(imgR.empty()){
	cout << "Failed to capture an image on camera 2" << endl;
	return -1;
      }

      oddBot.updateBoxPos(imgL, imgR,&xRobot,&yRobot,&tRobot);
      perc_track->robotPose.x = xRobot;
      perc_track->robotPose.y = yRobot;
      perc_track->robotPose.theta = tRobot;
      rPub.publish(perc_track->robotPose);
      r.sleep();
      ros::spinOnce();
    }

  return 0;
}
