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
  if (argc != 2) {
    printf("## You need to supply a number for the image to test ##\n");
    return -1;
  }

  //-- Initialize the first car location
  Point upperLeft(540,295);
  Point lowerRight(683,468);

  //-- Initialize a car object 
  Bot oddBot(upperLeft,lowerRight);

  //-- Iterate through images 
  //for (int j = 1; j < 252; j++)
  //  {
  
  int j = atoi(argv[1]);
  

  char imName1[30];
  char imName2[30];
  sprintf(imName1,"images/test_images/left%03d.jpg",j);
  sprintf(imName2,"images/test_images/right%03d.jpg",j);
  Mat imgL = imread( imName1, CV_LOAD_IMAGE_COLOR );
  Mat imgR = imread( imName2, CV_LOAD_IMAGE_COLOR );
  printf("%s\n",imName1);
  if(!imgL.data || !imgR.data)                              // Check for invalid input
    {
      printf("## Could not open or find the image ##\n") ;
        return -1;
    }
  //waitKey(10);
  //firstCar.updateBoxSize(img1);

  oddBot.updateBoxPos(imgL, imgR);
  waitKey(10);
      // }
  return 0;
}

