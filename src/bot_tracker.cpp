//This file uses the Car class to track a vehicle in a set of images
//Change this file to track a different car in a different image
//Although the robustness of the Car class is not guaranteed!
//
//Written by Eric Danziger
//04 March 2015
//ericdanziger@cmu.edu

#include "bot.h"
using namespace cv;

int main( int argc, char** argv )
{
  //-- Initialize the first car location
  Point upperLeft(540,295);
  Point lowerRight(683,468);
   
  //Mat initialImg = imread("images/left012.jpg", CV_LOAD_IMAGE_GRAYSCALE );
  //if(!initialImg.rows)
  //  {
    //   printf("This program expects files in '../car/'\n");
    //   return 0;
    // }
  //-- Initialize a car object 
  Bot oddBot(upperLeft,lowerRight);

  //-- Iterate through images 
  //for (int j = 1; j < 252; j++)
  //  {
  int j = 36;
  char imName1[30];
  char imName2[30];
  sprintf(imName1,"images/test_images/left%03d.jpg",j);
  sprintf(imName2,"images/test_images/right%03d.jpg",j);
  Mat imgL = imread( imName1, CV_LOAD_IMAGE_GRAYSCALE );
  Mat imgR = imread( imName2, CV_LOAD_IMAGE_GRAYSCALE );
  printf("%s\n",imName1);
  //waitKey(10);
  //firstCar.updateBoxSize(img1);

  oddBot.updateBoxPos(imgL, imgR);

      // }
  return 0;
}

