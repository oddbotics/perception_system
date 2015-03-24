#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>


using namespace cv;
using namespace std;

int main()
{
  VideoCapture capture(1); // open the video file for reading
  capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);

  if(!capture.isOpened()){
    cout << "Failed to connect to the camera." << endl;
  }
  Mat frame, edges;
  capture >> frame;
  if(frame.empty()){
    cout << "Failed to capture an image" << endl;
    return -1;
  }

  //cvtColor(frame, edges, CV_BGR2GRAY);
  //Canny(edges, edges, 0, 30, 3);
  imshow("edges", edges);
  imshow("capture", frame);

  return 0;
}
