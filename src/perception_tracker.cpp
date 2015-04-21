/*
 * Peception tracker for Robot
 *
 * By: Eric Danziger
 * March 1, 2015
 */


#include "perception_system/perception_tracker.h"
#include "perception_system/bot.h"


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
  int showI;
  if(argc < 2)
    showI = 0;
  else
    showI = atoi(argv[1]);
  Bot *oddBot = new Bot(showI);
  
  float xRobot,yRobot,tRobot;
  float xGoal,yGoal,tGoal;
  float posError;

  perceptionTracker *perc_track= new perceptionTracker();

  ros::Publisher rPub = nh.advertise<geometry_msgs::Pose2D>("/robot_perceived_pose",10);
  ros::Publisher gPub = nh.advertise<geometry_msgs::Pose2D>("/goal_perceived_pose",1,true);

  perc_track->goalPose.x = 1.9;
  perc_track->goalPose.y = 2.5;
  gPub.publish(perc_track->goalPose);

  ros::Rate r(100);

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
    std::cout << "Failed to connect to the camera 1." << std::endl;
  }
  if(!capture2.isOpened()){
    std::cout << "Failed to connect to the camera 2." << std::endl;
  }    


  while(nh.ok())
    {
      //Do the OPENCV stuff

      cv::Mat imgL,imgR;
      capture1>>imgL;
      capture2>>imgR;

      // while(!imgL.empty())
      for(int i = 0; i < 100; i++)
	{
	  capture1>>imgL;
	  capture2>>imgR;
	  waitKey(1);
	}
      waitKey(1);
      capture1>>imgL;
      capture2>>imgR;

      if(imgL.empty()){
	std::cout << "Failed to capture an image on camera 1" << std::endl;
	return -1;
      }
      if(imgR.empty()){
	std::cout << "Failed to capture an image on camera 2" << std::endl;
	return -1;
      }
      // cv::imshow("Left Image - ", imgL);
      // waitKey(1000);
      oddBot->updateBoxPos(imgL, imgR,&xRobot,&yRobot,&tRobot);
      perc_track->robotPose.x = xRobot;
      perc_track->robotPose.y = yRobot;
      perc_track->robotPose.theta = tRobot;
      rPub.publish(perc_track->robotPose);
      
      if(argc > 2)
	{
	  oddBot->findGoalPos(imgL, imgR,&xGoal,&yGoal,&tGoal);
	  perc_track->goalPose.x = xGoal;
	  perc_track->goalPose.y = yGoal;
	  perc_track->goalPose.theta = tGoal;
	  gPub.publish(perc_track->goalPose);
      	  
	}
      
      posError = sqrt(pow((perc_track->goalPose.x - perc_track->robotPose.x),2) + pow((perc_track->goalPose.y - perc_track->robotPose.y),2));
      printf("Current robot distance from goal: %f\n",posError);
      if(posError < .1)
	{
	  printf("\n#------- GOAL ACHIEVED!!!!! -----------#\n");
	  return 0;
	}
      r.sleep();
      ros::spinOnce();
    }

  return 0;
}
