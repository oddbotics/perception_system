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
  VideoCapture capture1(1); // open the video file for reading
  VideoCapture capture2(2); 
  //VideoCapture capture1(1); // open the video file for reading
  //capture0.set(CV_CAP_PROP_FRAME_WIDTH,640);
  //capture0.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  capture1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  capture1.set(CV_CAP_PROP_FRAME_HEIGHT,720);
  capture2.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  capture2.set(CV_CAP_PROP_FRAME_HEIGHT,720);

  int key = 0;
  int n;
  char buffer[25];
  Mat frame1,frame2;
  //while(key != 'q')
  for(int i = 0; i < 100; i++)
   {
  
  if(!capture1.isOpened()){
    cout << "Failed to connect to the camera 1." << endl;
  }
  if(!capture2.isOpened()){
    cout << "Failed to connect to the camera 2." << endl;
  }    

  capture1 >> frame1;
  //waitKey(10);
  capture2 >> frame2;
  //waitKey(10);
  if(frame1.empty()){
    cout << "Failed to capture an image on camera 1" << endl;
    return -1;
  }

  if(frame2.empty()){
    cout << "Failed to capture an image on camera 2" << endl;
    return -1;
  }


    
  vector<Mat> leftImages, rightImages;
  leftImages.push_back(frame1);
  
  rightImages.push_back(frame2);

  imshow("leftImage",frame1);
  imshow("rightImage",frame2);
  
  n = sprintf(buffer,"images/left%03d.jpg",i);
  imwrite(buffer,frame1);

  n = sprintf(buffer,"images/right%03d.jpg",i);
  imwrite(buffer,frame2);

  waitKey(1);
   }
  
  //start to compare the images


  //-- Detect the keypoints using SURF Detector
  // int minHessian = 400;

  // SurfFeatureDetector detector( minHessian );

  // std::vector<KeyPoint> keypoints1, keypoints2, keypointsInBox1, keypointsGM1, keypointsGM2;

  // detector.detect( frame1, keypoints1 );
  // detector.detect( frame2, keypoints2 );

  // Mat imgKeypoints1; Mat imgKeypoints2;
  
  //-- Detect keypoints that are within the box 
 
  // for (int i = 0; i < keypoints1.size(); i++)
  //   {
  //     if (keypoints1[i].pt.x < lowerRight.x && keypoints1[i].pt.x > upperLeft.x &&
  // 	  keypoints1[i].pt.y < lowerRight.y && keypoints1[i].pt.y > upperLeft.y)
  // 	{
  // 	  keypointsInBox1.push_back(keypoints1[i]);
  // 	}

  //   }
  // //-- Match points from within the box to the next frame
  // matchPoints(&keypointsInBox1,&keypoints2,image1,image2,&keypointsGM1,&keypointsGM2,2.1);


  //key = waitKey();
  // cout << key << endl;
    

  return 0;
}
