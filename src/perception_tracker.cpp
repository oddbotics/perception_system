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

  ros::Publisher relPub = nh.advertise<nav_msgs::Path>("/path",10);
  
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

      //-- Calculate In Robot Frame
      float radius = sqrt( pow(perc_track->goalPose.x - perc_track->robotPose.x,2) +  pow(perc_track->goalPose.y - perc_track->robotPose.y,2));
      //Is y or x on top?
      float phi = atan2( (perc_track->goalPose.y - perc_track->robotPose.y) , (perc_track->goalPose.x - perc_track->robotPose.x) );
      if(phi < 0)
	phi = 6.28 + phi;
      //Is this order correct?
      float phi2 =  phi - (perc_track->robotPose.theta*3.14)/180.0;

      if (phi2 < 0)
	phi2 = 2*3.14 - phi;
      
      printf("changed angles: %f %f\n",phi,phi2);

      printf("alternate phi: %f\n",atan2( (perc_track->goalPose.x - perc_track->robotPose.x) , (perc_track->goalPose.y - perc_track->robotPose.y) ));

      printf("radius: %f\n",radius);

      float ratio = tan(phi2);

      //float y2 = sqrt( pow(radius,2) / (1 + pow(ratio,2)));

      //float x2 = sqrt( pow(radius,2) - pow(y2,2) );

      float y2 = radius * sin(phi2);
      
      float x2 = radius * cos(phi2);

      geometry_msgs::Pose2D relativePose;
      relativePose.x = x2;
      relativePose.y = y2;
      
      geometry_msgs::PoseStamped relPose;
      relPose.pose.position.x = x2;
      relPose.pose.position.y = y2;

      nav_msgs::Path relPath;
      relPath.poses.push_back(relPose);

      relPub.publish(relPath);
      
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
