//Written by Eric Danziger
//04 March 2015
//ericdanziger@cmu.edu

#ifndef BOT_H
#define BOT_H


#include <dirent.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <numeric>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;

class Bot
{
 public:
  Bot();
  Bot(int);
  Bot(Point uL, Point lR);
  ~Bot();
  
  void updateBoxPos(Mat image1, Mat image2, float*, float*, float*);
  void findGoalPos(Mat image1, Mat image2, float*, float*, float*);
  void updateBoxSize(Mat image);
  bool unitTest(Mat image);

 private:
  bool showImages;
  Point upperLeft;
  Point lowerRight;
  bool inertialTracking; 
  Mat templateIm;
  int updateTemplate;
 
  std::vector<Mat> confirmedFeatures;
  std::vector<int> gradientHistory;
  std::vector<int> xHistory;
  std::vector<int> yHistory;

  void checkBounds(Mat image, Point* uL, Point* lR);
  void getTemplateMatch(Mat image, Point* matchUL, Point* matchLR, Mat* templateImage, double* strongestResponse);
  void matchPoints(std::vector<KeyPoint>* keypointsIn1, std::vector<KeyPoint>* keypointsIn2,Mat image1, Mat image2, std::vector<KeyPoint>* keypointsOut1, std::vector<KeyPoint>* keypointsOut2, int thresh);
  
  

};


#endif
