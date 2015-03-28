#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>


using namespace cv;
using namespace std;

int main()
{
  VideoCapture capture1(0); // open the video file for reading
  //VideoCapture capture1(1); // open the video file for reading
  //capture0.set(CV_CAP_PROP_FRAME_WIDTH,640);
  //capture0.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  capture1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  capture1.set(CV_CAP_PROP_FRAME_HEIGHT,720);

  int key = 0;
  // while(key != 'q')
  //   {
  //   }
  Mat frame;
 
  if(!capture1.isOpened()){
    cout << "Failed to connect to the camera." << endl;
  }
      
  capture1 >> frame;

  if(frame.empty()){
    cout << "Failed to capture an image" << endl;
    return -1;
  }

    
  vector<Mat> leftImages, rightImages;
  leftImages.push_back(frame);
  waitKey(10);
  capture1>>frame;
  if(frame.empty()){
    cout << "Failed to capture an image" << endl;
    return -1;
  }
  rightImages.push_back(frame);

  imshow("leftImage",leftImages[0]);
  imshow("rightImage",rightImages[0]); 

  //start to compare the images


  //-- Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints1, keypoints2, keypointsInBox1, keypointsGM1, keypointsGM2;

  detector.detect( image1, keypoints1 );
  detector.detect( image2, keypoints2 );

  Mat imgKeypoints1; Mat imgKeypoints2;
  
  //-- Detect keypoints that are within the box 
 
  for (int i = 0; i < keypoints1.size(); i++)
    {
      if (keypoints1[i].pt.x < lowerRight.x && keypoints1[i].pt.x > upperLeft.x &&
	  keypoints1[i].pt.y < lowerRight.y && keypoints1[i].pt.y > upperLeft.y)
	{
	  keypointsInBox1.push_back(keypoints1[i]);
	}

    }
  //-- Match points from within the box to the next frame
  matchPoints(&keypointsInBox1,&keypoints2,image1,image2,&keypointsGM1,&keypointsGM2,2.1);


  key = waitKey();
  // cout << key << endl;
    

  return 0;
}
