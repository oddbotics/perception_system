//This is the Car class
//The main focus is on two public functions to update the position and size of the tracking box
//The updateBoxSize function can be modified to be more robust for other image sets
//
//Written by Eric Danziger
//04 March 2015
//ericdanziger@cmu.edu

#include "perception_system/bot.h"
using namespace cv;

Bot::Bot()
{
  upperLeft.x = 1;
  upperLeft.y = 1;
  lowerRight.x = 100;
  lowerRight.y = 100;
  showImages = false;
  //checkBounds(initialIm, &upperLeft, &lowerRight);
  //templateIm = initialIm(Range(upperLeft.y,lowerRight.y),Range(upperLeft.x,lowerRight.x));
  updateTemplate = 1;  
}


Bot::Bot(int sI)
{
  if(sI == 1)
    showImages = true;
  else
    showImages = false;
  upperLeft.x = 1;
  upperLeft.y = 1;
  lowerRight.x = 100;
  lowerRight.y = 100;
  inertialTracking = false;
  //checkBounds(initialIm, &upperLeft, &lowerRight);
  //templateIm = initialIm(Range(upperLeft.y,lowerRight.y),Range(upperLeft.x,lowerRight.x));
  updateTemplate = 1;  
}

Bot::Bot(Point uL, Point lR)
{
  upperLeft = uL;
  lowerRight = lR;
  inertialTracking = false;
  //checkBounds(initialIm, &upperLeft, &lowerRight);
  //templateIm = initialIm(Range(upperLeft.y,lowerRight.y),Range(upperLeft.x,lowerRight.x));
  updateTemplate = 1;  
}

Bot::~Bot()
{
}

void Bot::matchPoints(std::vector<KeyPoint>* keypointsIn1, std::vector<KeyPoint>* keypointsIn2,Mat image1, Mat image2, std::vector<KeyPoint>* keypointsOut1, std::vector<KeyPoint>* keypointsOut2, int thresh)
{
  if(!keypointsIn1->size() || !keypointsIn2->size())
    {
      //-- Detect the keypoints using SURF Detector
      int minHessian = 400;

      SurfFeatureDetector detector( minHessian );
  
      detector.detect( image1, *keypointsIn1 );
      detector.detect( image2, *keypointsIn2 );
    }

  //-- Extract features for matching
  SurfDescriptorExtractor extractor;

  Mat descriptors1, descriptors2;

  extractor.compute(image1, *keypointsIn1, descriptors1);
  extractor.compute(image2, *keypointsIn2, descriptors2);

  FlannBasedMatcher matcher;
  std::vector<DMatch> matches;

  //-- Match the car SURF points with all the next images SURF points
  matcher.match(descriptors1,descriptors2, matches);

  //-- Find match strengths
  double maxDist = 0; double minDist = 100;

  for( int i = 0; i < descriptors1.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < minDist ) minDist = dist;
      if( dist > maxDist ) maxDist = dist;
    }
  
  //-- Keep good matches
  std::vector< DMatch > goodMatches;
  
  for( int i = 0; i < descriptors1.rows; i++ )
    { if( matches[i].distance <= max(thresh*minDist, 0.02) )
	{ goodMatches.push_back( matches[i]); }
    }

  //-- Fill the good matches keypoint vectors
  for ( int i = 0; i < goodMatches.size(); i++)
    {
      (*keypointsOut1).push_back((*keypointsIn1)[goodMatches[i].queryIdx]);     
      (*keypointsOut2).push_back((*keypointsIn2)[goodMatches[i].trainIdx]);
    }

}

void Bot::findGoalPos(Mat imageL, Mat imageR, float *xGoal, float *yGoal, float *tGoal)
{
  double lBestResponse=100;
  int lMostPoints=0;
  int lRotI;
  float lBestDeg, lScale;
  Point lBestUL,lBestLR;

  double rBestResponse=100;
  int rRotI;
  float rBestDeg, rScale;
  Point rBestUL,rBestLR;
  
  //-- Check left image for goal
  
  for (float scale=.7;scale < 2.00; scale=scale+.10)
    {
      Point locUL,locLR;
      double sResponse;
      float deg;
      char imName[30];
      Mat tempIm,tempEdges,lTempEdges;
      Size scaleSize;
      sprintf(imName,"images/templates/20APR/leftGoal.jpg");
      //printf("%s %f \n",imName,scale);
      tempIm = imread(imName, CV_LOAD_IMAGE_COLOR);
      scaleSize.height = tempIm.rows*scale;
      scaleSize.width = tempIm.cols*scale;        
      resize(tempIm,tempIm,scaleSize);
	 
      //Do it on Edges
      //blur(tempIm,tempEdges,Size(3,3));
      //Canny(tempEdges,tempEdges,120,140,3);
	  
      //Mat halfImage = lTempEdges(Range(imageL.cols/2,imageL.cols),Range(1,imageL.rows));
      Mat halfImage = imageL(Range(imageL.rows/2,imageL.rows),Range(1,imageL.cols));

      //blur(halfImage,lTempEdges,Size(3,3));
      //Canny(lTempEdges,lTempEdges,120,140,3);
	  
      //templateIm = initialIm(Range(upperLeft.y,lowerRight.y),Range(upperLeft.x,lowerRight.x));
      if(showImages)
	{
	  imshow("goalTemplate",tempIm);
	  moveWindow("goalTemplate",10,500);
	  //imshow("edges",lTempEdges);
	  waitKey(10);
	}	 
      
      getTemplateMatch(imageL,&locUL,&locLR, &tempIm, &sResponse);
      deg = 0;
	  
      Mat imageS = imageL.clone();
      rectangle(imageS,locUL,locLR,10,3);
      //sResponse = sResponse/(8900*pow(scale,2));
      if (lBestResponse < sResponse)
	{
	  lScale = scale;
	  lRotI = 1;
	  lBestUL = locUL;
	  lBestLR = locLR;
	  lBestResponse = sResponse;
	  lBestDeg = deg;
	  printf("lBest Response: %f\n",lBestResponse);
	}
      if(showImages)
	{
	  rectangle(imageS,lBestUL,lBestLR,205,3);
	  imshow("Left Image Goal",imageS);
	  moveWindow("Left Image Goal",200,400);
	  waitKey(10);
	}
    }
  

  //-- Check right image for goal
  for (float scale=.7;scale < 2.00; scale=scale+.10)
    {
      Point locUL,locLR;
      double sResponse;
      float deg;
      char imName[30];
      Mat tempIm,tempEdges,rTempEdges;
      Size scaleSize;
      sprintf(imName,"images/templates/20APR/rightGoal.jpg");
      //printf("%s %f \n",imName,scale);
      tempIm = imread(imName, CV_LOAD_IMAGE_COLOR);
      scaleSize.height = tempIm.rows*scale;
      scaleSize.width = tempIm.cols*scale;
	  
      resize(tempIm,tempIm,scaleSize);
	 
      //Do it on Edges
      //blur(tempIm,tempEdges,Size(3,3));
      //Canny(tempEdges,tempEdges,100,120,3);

      //blur(imageR,rTempEdges,Size(3,3));
      //Canny(rTempEdges,rTempEdges,100,120,3);

      Mat halfImage = imageR(Range(imageR.rows/2,imageR.rows),Range(1,imageR.cols));
      if(showImages)
	{
	  imshow("goalTemplate",tempIm);
	  //waitKey(10);
	}
      getTemplateMatch(imageR,&locUL,&locLR, &tempIm, &sResponse);
      deg = 0;
      //printf("LEFT\ndegree: %f \ntemplate: %d \nlocUL-x: %d \nlocUL-y: %d \nlocLR-x: %d\nlocLR-y: %d\n",deg,i,locUL.x,locUL.y,locLR.x,locLR.y);
      Mat imageS = imageR.clone();
      rectangle(imageS,locUL,locLR,10,3);
      //sResponse = sResponse/(8900*pow(scale,2));
      if (rBestResponse < sResponse)
	{
	  rScale = scale;
	  rRotI = 1;
	  rBestUL = locUL;
	  rBestLR = locLR;
	  rBestResponse = sResponse;
	  rBestDeg = deg;
	  printf("rBest Response: %f\n",rBestResponse);
	}
      if(showImages)
	{
	  rectangle(imageS,rBestUL,rBestLR,200,3);
	  imshow("Right Image Goal",imageS);
	  moveWindow("Right Image Goal",200,500);
	}
      //rectangle(imgKeypoints1,upperLeft,lowerRight,10,3);
      waitKey(10);
    }
    

  //-- Estimate a box 

  //-- Match points inside the boxes in the Left and Right images
  //  int minHessian = 400;

 //  SurfFeatureDetector detector( minHessian );

 //  std::vector<KeyPoint> keypointsL, keypointsR, keypointsBotL, keypointsBotR, keypointsGML, keypointsGMR;

 //  detector.detect( imageL, keypointsL );
 //  detector.detect( imageR, keypointsR );

 //  Mat imgKeypointsL; Mat imgKeypointsR;
  
 //  //-- Detect keypoints that are within the box -LEFT

 //  Mat halfImage = imageR(Range(imageR.rows/2,imageR.rows),Range(1,imageR.cols));
	  
  
 //  for (int i = 0; i < keypointsL.size(); i++)
 //    {
 //      if (keypointsL[i].pt.x < lBestLR.x && keypointsL[i].pt.x > lBestUL.x &&
 // 	  keypointsL[i].pt.y < (lBestLR.y /*+ imageL.rows/2 */) && keypointsL[i].pt.y > (lBestUL.y /*+ imageL.rows/2*/))
 // 	{
 // 	  keypointsBotL.push_back(keypointsL[i]);
 // 	}
 //    }
 // //-- Detect keypoints that are within the box - RIGHT
 
 // for (int i = 0; i < keypointsR.size(); i++)
 //    {
 //      if (keypointsR[i].pt.x < rBestLR.x && keypointsR[i].pt.x > rBestUL.x &&
 // 	  keypointsR[i].pt.y < (rBestLR.y /*+ imageR.rows/2*/) && keypointsR[i].pt.y > (rBestUL.y /*+ imageR.rows/2*/))
 // 	{
 // 	  keypointsBotR.push_back(keypointsR[i]);
 // 	}
 //    }
 //  //-- Match points from within the box to the next frame
 //  matchPoints(&keypointsBotL,&keypointsBotR,imageL,imageR,&keypointsGML,&keypointsGMR,5.1);
  
  
  //-- Triangulate distance
  
  // Mat lProj,rProj;
 //  string paramsyml = "projmats.yml";
 //  FileStorage fsP(paramsyml,FileStorage::READ);
 //  fsP["P1"] >>lProj;
 //  fsP["P2"] >>rProj;

 //  Mat worldPoints,imageWithKeyPointsL,imageWithKeyPointsR;
 //  std::vector<Point2f> lPoints,rPoints;
 //  for (int i = 0; i < keypointsGML.size(); i++)
 //    {
 //      lPoints.push_back(keypointsGML[i].pt);
 //    }
 // for (int i = 0; i < keypointsGMR.size(); i++)
 //    {
 //      rPoints.push_back(keypointsGMR[i].pt);
 //    }
 // drawKeypoints( imageL, keypointsGML, imageWithKeyPointsL, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
 // drawKeypoints( imageR, keypointsGMR, imageWithKeyPointsR, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

 // //imshow("With Keypoints L - GOAL", imageWithKeyPointsL);
 // //imshow("With Keypoints R - GOAL", imageWithKeyPointsR);
 // waitKey(10);

 // Point2f tempPoint;
 // //left 0,0
 // tempPoint.x = 206;
 // tempPoint.y = 568;
 // lPoints.push_back(tempPoint);
 // //left 0,10
 // tempPoint.x = 252;
 // tempPoint.y = 409;
 // lPoints.push_back(tempPoint);
 // //left 10,0
 // tempPoint.x = 1134;
 // tempPoint.y = 502;
 // lPoints.push_back(tempPoint);
 // //left 10,10
 // tempPoint.x = 762;
 // tempPoint.y = 395;
 // lPoints.push_back(tempPoint);

 // //right 0,0
 // tempPoint.x = 71;
 // tempPoint.y = 636;
 // rPoints.push_back(tempPoint);
 // //right 0,10
 // tempPoint.x = 169;
 // tempPoint.y = 474;
 // rPoints.push_back(tempPoint);
 // //right 10,0
 // tempPoint.x = 1018;
 // tempPoint.y = 545;
 // rPoints.push_back(tempPoint);
 // //right 10,10
 // tempPoint.x = 689;
 // tempPoint.y = 449;
 // rPoints.push_back(tempPoint);

 // //std::cout << "\n2D POINTS" << std::endl << lPoints << std::endl << rPoints <<std::endl;
 
 // triangulatePoints(lProj,rProj,lPoints,rPoints,worldPoints);
 //  //triangulatePoints(lProj2,rProj2,lPoints,rPoints,worldPoints);

 // //-- Normalize by the scale factor
 // //-- And get the average of the values
 // //-- Make sure to not include the four boundary points in the average
 //  std::vector<float> x,y,z;
 //  for (int i = 0; i < (worldPoints.cols - 4); i++)
 //    {
 //      worldPoints.at<float>(0,i) /= worldPoints.at<float>(3,i);
 //      x.push_back(worldPoints.at<float>(0,i));
 //      worldPoints.at<float>(1,i) /= worldPoints.at<float>(3,i);
 //      y.push_back(worldPoints.at<float>(1,i));
 //      worldPoints.at<float>(2,i) /= worldPoints.at<float>(3,i);
 //      z.push_back(worldPoints.at<float>(2,i));
 //      worldPoints.at<float>(3,i) /= worldPoints.at<float>(3,i);
 //    }

 // float avgX = std::accumulate(x.begin(), x.end(), 0) / (float)x.size();
 // float avgY = std::accumulate(y.begin(), y.end(), 0) / (float)y.size();
 // float avgZ = std::accumulate(z.begin(), z.end(), 0) / (float)z.size();

 // //std::cout<< "\nworldPoints: "<< std::endl << worldPoints << std::endl;
 //  // printf("WorldPoints: \n",worldPoints);
 //  // printf("Avg X: %f\n",avgX);
 //  // printf("Avg Y: %f\n",avgY);
 //  // printf("Avg Z: %f\n",avgZ);

 //  for (int i = (worldPoints.cols - 4); i < worldPoints.cols; i++)
 //    {
 //      //printf("i: %d\n",i);
 //      worldPoints.at<float>(0,i) /= worldPoints.at<float>(3,i);
 //      //printf("X:  %f\n",worldPoints.at<float>(0,i));
 //      worldPoints.at<float>(1,i) /= worldPoints.at<float>(3,i);
 //      //printf("Y:  %f\n",worldPoints.at<float>(1,i));
 //      worldPoints.at<float>(2,i) /= worldPoints.at<float>(3,i);
 //      //printf("Z:  %f\n",worldPoints.at<float>(2,i));
 //      worldPoints.at<float>(3,i) /= worldPoints.at<float>(3,i);
 //    }

 //  Mat boundW = worldPoints(Range(0,(worldPoints.rows-1)),Range(worldPoints.cols-4,worldPoints.cols));

  //std::cout << std::endl << boundW << std::endl;

  //-- Map it to something based on the boundary values
  if(lBestDeg == 0 && rBestDeg > 180)
    {
      lBestDeg = 360.0;
    }
  if(rBestDeg == 0 && lBestDeg > 180)
    {
      rBestDeg = 360.0;
    }
      
  //  printf("\nScale: %f\n",(lScale + rScale)/2);

  //printf("leftLoc %d %d\n",(lBestUL.x + lBestLR.x)/2,(lBestUL.y + lBestLR.y)/2);
  //printf("rightLoc %d %d\n",(rBestUL.x + rBestLR.x)/2,(rBestUL.y + rBestLR.y)/2);
  
  // Mat boundP = Mat::zeros(3, 4, CV_32F);

  // boundP.at<float>(1,1) = 10.0;
  // boundP.at<float>(0,2) = 10.0;
  // boundP.at<float>(0,3) = 10.0;
  // boundP.at<float>(1,3) = 10.0;

  // //std::cout << boundP << std::endl;

  // Mat warp_mat( 3, 4, CV_32FC1 );
  //warp_mat = getAffineTransform( boundW,boundP);
  //std::vector<T>::const_iterator first  = worldPoints.end() - 4;
  // std::vector<T>::const_iterator last = worldPoints.end();
  // std::vector<T> inBound(first, last);

 if(lBestDeg == 0 && rBestDeg > 180)
    {
      lBestDeg = 360.0;
    }
  if(rBestDeg == 0 && lBestDeg > 180)
    {
      rBestDeg = 360.0;
    }

  float bestDeg = (lBestDeg + rBestDeg)/2 + 45.0;
  if (bestDeg > 360)
    {
      bestDeg = bestDeg - 360.0;
    }
  printf("\nScale: %f\n",(lScale + rScale)/2);

  printf("leftLoc %d %d\n",(lBestUL.x + lBestLR.x)/2,(lBestUL.y + lBestLR.y)/2);
  printf("rightLoc %d %d\n",(rBestUL.x + rBestLR.x)/2,(rBestUL.y + rBestLR.y)/2);


  printf("\nBest Estimate of Rotation: %f\n",bestDeg);

  //-- Translate scale and position
  float yTrans = 5.8365 - 3.3654*(lScale + rScale)/2;
  float xLeftTrans = ((lBestUL.x + lBestLR.x)/2 - 250)*(4.25/(1100-250));
  float xRightTrans = ((rBestUL.x + rBestLR.x)/2 - 150)*(4.25/(1000-150));

  *yGoal = yTrans;
  *xGoal = (xLeftTrans+xRightTrans)/2;
  *tGoal = bestDeg;

  printf("Estimate of X,Y pos of GOAL: %f %f\n",(xLeftTrans+xRightTrans)/2,yTrans);
    
  
  waitKey(1);  

}

void Bot::updateBoxPos(Mat imageL, Mat imageR, float *xBot, float *yBot, float *tBot)
{
  double lBestResponse=100;
  int lMostPoints=0;
  int lRotI;
  float lBestDeg, lScale;
  Point lBestUL,lBestLR;

  double rBestResponse=100;
  int rRotI;
  float rBestDeg, rScale;
  Point rBestUL,rBestLR;
  
  //-- Check left image for robot 
  for (int i=1;i<9;i++)
    {
      for (float scale=.6;scale < 2.00; scale=scale+.15)
	{
	  Point locUL,locLR;
	  double sResponse;
	  float deg;
	  char imName[30];
	  Mat tempIm,tempEdges,lTempEdges;
	  Size scaleSize;
	  sprintf(imName,"images/templates/20APR/left%03d.jpg",i);
	  //printf("%s %f \n",imName,scale);
	  tempIm = imread(imName, CV_LOAD_IMAGE_COLOR);
	  scaleSize.height = tempIm.rows*scale;
	  scaleSize.width = tempIm.cols*scale;

	  //printf("scale height %d width %d\n",scaleSize.height,scaleSize.width); 
	  sprintf(imName,"%f d",(i*360.0/8));
	  
	  resize(tempIm,tempIm,scaleSize);
	 
	  //Do it on Edges
	  //blur(tempIm,tempEdges,Size(3,3));
	  //Canny(tempEdges,tempEdges,120,140,3);
	  
	  //Mat halfImage = lTempEdges(Range(imageL.cols/2,imageL.cols),Range(1,imageL.rows));
	  Mat halfImage = imageL(Range(imageL.rows/2,imageL.rows),Range(1,imageL.cols));

	  //blur(halfImage,lTempEdges,Size(3,3));
	  //Canny(lTempEdges,lTempEdges,120,140,3);
	  
	  //templateIm = initialIm(Range(upperLeft.y,lowerRight.y),Range(upperLeft.x,lowerRight.x));
	  if(showImages)
	    {
	      imshow("template",tempIm);
	      moveWindow("template",10,10);
	      //imshow("edges",lTempEdges);
	      waitKey(1);
	    }	  

	  getTemplateMatch(halfImage,&locUL,&locLR, &tempIm, &sResponse);
	  // getTemplateMatch(lTempEdges,&locUL,&locLR, &tempEdges, &sResponse);
	  // getTemplateMatch(imageL,&locUL,&locLR, &tempIm, &sResponse);
	  //locUL.y = locUL.y + imageL.rows/2;
	  //locLR.y = locLR.y + imageL.rows/2;
	  deg = (i-1)*(360.0/8);
	  //printf("LEFT\ndegree: %f \ntemplate: %d \nlocUL-x: %d \nlocUL-y: %d \nlocLR-x: %d\nlocLR-y: %d\n",deg,i,locUL.x,locUL.y,locLR.x,locLR.y);
	  
	  Mat imageS = halfImage.clone();
	  rectangle(imageS,locUL,locLR,10,3);
	  //sResponse = sResponse/(8900*pow(scale,2));
	  if (lBestResponse < sResponse)
	    {
	      lScale = scale;
	      lRotI = i;
	      lBestUL = locUL;
	      lBestLR = locLR;
	      lBestResponse = sResponse;
	      lBestDeg = deg;
	      //printf("lBest Response: %f\n",lBestResponse);
	    }
	  if(showImages)
	    {
	  rectangle(imageS,lBestUL,lBestLR,205,3);
	  imshow("Left Image",imageS);
	  moveWindow("Left Image",300,10);
	    }
	  //rectangle(imgKeypoints1,upperLeft,lowerRight,10,3);

	  //-- MATCHING SURF FEATURES

	  //-- Match points inside the boxes in the Left and template images
	  // int minHessian = 400;

	  // SurfFeatureDetector detector( minHessian );

	  // std::vector<KeyPoint> keypointsL, keypointsT, keypointsBotL, keypointsBotT, keypointsGML, keypointsGMT;

	  // detector.detect( halfImage, keypointsL );
	  // detector.detect( tempIm, keypointsT );

	  // Mat imgKeypointsL; Mat imgKeypointsT;
  
	  // //-- Detect keypoints that are within the box -LEFT
 
	  // for (int i = 0; i < keypointsL.size(); i++)
	  //   {
	  //     if (keypointsL[i].pt.x < lBestLR.x && keypointsL[i].pt.x > lBestUL.x &&
	  // 	  keypointsL[i].pt.y < lBestLR.y && keypointsL[i].pt.y > lBestUL.y)
	  // 	{
	  // 	  keypointsBotL.push_back(keypointsL[i]);
	  // 	}
	  //   }
	  // //-- Detect keypoints that are within the box - RIGHT
 
	  // // for (int i = 0; i < keypointsR.size(); i++)
	  // //   {
	  // //     if (keypointsR[i].pt.x < rBestLR.x && keypointsR[i].pt.x > rBestUL.x &&
	  // // 	  keypointsR[i].pt.y < rBestLR.y && keypointsR[i].pt.y > rBestUL.y)
	  // // 	{
	  // // 	  keypointsBotR.push_back(keypointsR[i]);
	  // // 	}
	  // //   }

	  // //-- Match points from within the box to the next frame
	  // matchPoints(&keypointsL,&keypointsT,halfImage,tempIm,&keypointsGML,&keypointsGMT,3.1);
	 
	  // drawKeypoints( halfImage, keypointsGML, imgKeypointsL, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	  // imshow("with Keypoints",imgKeypointsL);
	  // printf("Matched %d points\n",keypointsGML.size());
	  // if(keypointsGML.size()>lMostPoints)
	  //   {
	  //     lMostPoints = keypointsGML.size();
	  //     printf("---Highest Match is now %d\n",lMostPoints);
	  //     printf("---Highest Match Rotation is %f\n",deg);
	  //   }

	  //waitKey(1);
	}
    }
  
  printf("\nLBest Estimate of Left Rotation: %f\n",lBestDeg);

  //-- Check right image for robot 
  for (int i=1;i<9;i++)
    {
      for (float scale=.6;scale < 2.00; scale=scale+.15)
	{
	  Point locUL,locLR;
	  double sResponse;
	  float deg;
	  char imName[30];
	  Mat tempIm,tempEdges,rTempEdges;
	  Size scaleSize;
	  sprintf(imName,"images/templates/20APR/right%03d.jpg",i);
	  //printf("%s %f \n",imName,scale);
	  tempIm = imread(imName, CV_LOAD_IMAGE_COLOR);
	  scaleSize.height = tempIm.rows*scale;
	  scaleSize.width = tempIm.cols*scale;

	  // printf("scale height %d width %d\n",scaleSize.height,scaleSize.width); 
	  sprintf(imName,"%f d",(i*360.0/8));
	  
	  resize(tempIm,tempIm,scaleSize);
	 
	  //Do it on Edges
	  //blur(tempIm,tempEdges,Size(3,3));
	  //Canny(tempEdges,tempEdges,100,120,3);


	  //blur(imageR,rTempEdges,Size(3,3));
	  //Canny(rTempEdges,rTempEdges,100,120,3);

	  Mat halfImage = imageR(Range(imageR.rows/2,imageR.rows),Range(1,imageR.cols));
	  if(showImages)
	    {
	      imshow("template",tempIm);
	      moveWindow("template",10,10);
	      waitKey(1);
	    }
	  getTemplateMatch(halfImage,&locUL,&locLR, &tempIm, &sResponse);
	  deg = (i-1)*(360.0/8);
	  //printf("LEFT\ndegree: %f \ntemplate: %d \nlocUL-x: %d \nlocUL-y: %d \nlocLR-x: %d\nlocLR-y: %d\n",deg,i,locUL.x,locUL.y,locLR.x,locLR.y);
	  Mat imageS = halfImage.clone();
	  rectangle(imageS,locUL,locLR,10,3);
	  //sResponse = sResponse/(8900*pow(scale,2));
	  if (rBestResponse < sResponse)
	    {
	      rScale = scale;
	      rRotI = i;
	      rBestUL = locUL;
	      rBestLR = locLR;
	      rBestResponse = sResponse;
	      rBestDeg = deg;
	      //printf("rBest Response: %f\n",rBestResponse);
	    }
	  if(showImages)
	    {
	      rectangle(imageS,rBestUL,rBestLR,200,3);
	      imshow("Right Image",imageS);
	      moveWindow("Right Image",300,250);
	      
	      waitKey(1);
	    }
	}
    }
  
  printf("\nBest Estimate of Right Rotation: %f\n",rBestDeg);

  //-- Estimate a box 

  //-- Match points inside the boxes in the Left and Right images
  //  int minHessian = 400;

 //  SurfFeatureDetector detector( minHessian );

 //  std::vector<KeyPoint> keypointsL, keypointsR, keypointsBotL, keypointsBotR, keypointsGML, keypointsGMR;

 //  detector.detect( imageL, keypointsL );
 //  detector.detect( imageR, keypointsR );

 //  Mat imgKeypointsL; Mat imgKeypointsR;
  
 //  //-- Detect keypoints that are within the box -LEFT

 //  Mat halfImage = imageR(Range(imageR.rows/2,imageR.rows),Range(1,imageR.cols));
	  
  
 //  for (int i = 0; i < keypointsL.size(); i++)
 //    {
 //      if (keypointsL[i].pt.x < lBestLR.x && keypointsL[i].pt.x > lBestUL.x &&
 // 	  keypointsL[i].pt.y < (lBestLR.y + imageL.rows/2) && keypointsL[i].pt.y > (lBestUL.y + imageL.rows/2))
 // 	{
 // 	  keypointsBotL.push_back(keypointsL[i]);
 // 	}
 //    }
 // //-- Detect keypoints that are within the box - RIGHT
 
 // for (int i = 0; i < keypointsR.size(); i++)
 //    {
 //      if (keypointsR[i].pt.x < rBestLR.x && keypointsR[i].pt.x > rBestUL.x &&
 // 	  keypointsR[i].pt.y < (rBestLR.y + imageR.rows/2) && keypointsR[i].pt.y > (rBestUL.y + imageR.rows/2))
 // 	{
 // 	  keypointsBotR.push_back(keypointsR[i]);
 // 	}
 //    }
  //-- Match points from within the box to the next frame
  //matchPoints(&keypointsBotL,&keypointsBotR,imageL,imageR,&keypointsGML,&keypointsGMR,5.1);
  
  
  //-- Triangulate distance
  
  // Mat lProj,rProj;
  // string paramsyml = "projmats.yml";
  // FileStorage fsP(paramsyml,FileStorage::READ);
  // fsP["P1"] >>lProj;
  // fsP["P2"] >>rProj;

  // std::cout<<"Printing contents of proj mats :"<<std::endl;
  // std::cout<<lProj<<std::endl<<std::endl;
  // std::cout<<rProj<<std::endl<<std::endl;


  // std::vector<Point2f> stLP,stRP;
  // string paramsyml = "standardPoints.yml";
  // FileStorage fsP(paramsyml,FileStorage::READ);
  // fsP["leftP"] >>stLP;
  // fsP["rightP"] >>stRP;

  // Mat worldPoints,imageWithKeyPointsL,imageWithKeyPointsR;
 //  std::vector<Point2f> lPoints,rPoints;
 //  for (int i = 0; i < keypointsGML.size(); i++)
 //    {
 //      lPoints.push_back(keypointsGML[i].pt);
 //    }
 // for (int i = 0; i < keypointsGMR.size(); i++)
 //    {
 //      rPoints.push_back(keypointsGMR[i].pt);
 //    }
 // drawKeypoints( imageL, keypointsGML, imageWithKeyPointsL, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
 // drawKeypoints( imageR, keypointsGMR, imageWithKeyPointsR, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

 // //imshow("With Keypoints L", imageWithKeyPointsL);
 // //imshow("With Keypoints R", imageWithKeyPointsR);
 // waitKey(10);

 // Point2f tempPoint;
 // //left 0,0
 // tempPoint.x = 206;
 // tempPoint.y = 568;
 // lPoints.push_back(tempPoint);
 // //left 0,10
 // tempPoint.x = 252;
 // tempPoint.y = 409;
 // lPoints.push_back(tempPoint);
 // //left 10,0
 // tempPoint.x = 1134;
 // tempPoint.y = 502;
 // lPoints.push_back(tempPoint);
 // //left 10,10
 // tempPoint.x = 762;
 // tempPoint.y = 395;
 // lPoints.push_back(tempPoint);

 // //right 0,0
 // tempPoint.x = 71;
 // tempPoint.y = 636;
 // rPoints.push_back(tempPoint);
 // //right 0,10
 // tempPoint.x = 169;
 // tempPoint.y = 474;
 // rPoints.push_back(tempPoint);
 // //right 10,0
 // tempPoint.x = 1018;
 // tempPoint.y = 545;
 // rPoints.push_back(tempPoint);
 // //right 10,10
 // tempPoint.x = 689;
 // tempPoint.y = 449;
 // rPoints.push_back(tempPoint);

 // //std::cout << "\n2D POINTS" << std::endl << lPoints << std::endl << rPoints <<std::endl;
 
 // triangulatePoints(lProj,rProj,lPoints,rPoints,worldPoints);
 //  //triangulatePoints(lProj2,rProj2,lPoints,rPoints,worldPoints);

 // //-- Normalize by the scale factor
 // //-- And get the average of the values
 // //-- Make sure to not include the four boundary points in the average
 //  std::vector<float> x,y,z;
 //  for (int i = 0; i < (worldPoints.cols - 4); i++)
 //    {
 //      worldPoints.at<float>(0,i) /= worldPoints.at<float>(3,i);
 //      x.push_back(worldPoints.at<float>(0,i));
 //      worldPoints.at<float>(1,i) /= worldPoints.at<float>(3,i);
 //      y.push_back(worldPoints.at<float>(1,i));
 //      worldPoints.at<float>(2,i) /= worldPoints.at<float>(3,i);
 //      z.push_back(worldPoints.at<float>(2,i));
 //      worldPoints.at<float>(3,i) /= worldPoints.at<float>(3,i);
 //    }

 //  float avgX = std::accumulate(x.begin(), x.end(), 0) / (float)x.size();
 //  float avgY = std::accumulate(y.begin(), y.end(), 0) / (float)y.size();
 //  float avgZ = std::accumulate(z.begin(), z.end(), 0) / (float)z.size();

 //  //std::cout<< "\nworldPoints: "<< std::endl << worldPoints << std::endl;
 //  // printf("WorldPoints: \n",worldPoints);
 //  // printf("Avg X: %f\n",avgX);
 //  // printf("Avg Y: %f\n",avgY);
 //  // printf("Avg Z: %f\n",avgZ);

 //  for (int i = (worldPoints.cols - 4); i < worldPoints.cols; i++)
 //    {
 //      //printf("i: %d\n",i);
 //      worldPoints.at<float>(0,i) /= worldPoints.at<float>(3,i);
 //      //printf("X:  %f\n",worldPoints.at<float>(0,i));
 //      worldPoints.at<float>(1,i) /= worldPoints.at<float>(3,i);
 //      //printf("Y:  %f\n",worldPoints.at<float>(1,i));
 //      worldPoints.at<float>(2,i) /= worldPoints.at<float>(3,i);
 //      // printf("Z:  %f\n",worldPoints.at<float>(2,i));
 //      worldPoints.at<float>(3,i) /= worldPoints.at<float>(3,i);
 //    }

 //  Mat boundW = worldPoints(Range(0,(worldPoints.rows-1)),Range(worldPoints.cols-4,worldPoints.cols));

  //std::cout << std::endl << boundW << std::endl;

  //-- Map it to something based on the boundary values
  if(lBestDeg == 0 && rBestDeg > 180)
    {
      lBestDeg = 360.0;
    }
  if(rBestDeg == 0 && lBestDeg > 180)
    {
      rBestDeg = 360.0;
    }

  float bestDeg = (lBestDeg + rBestDeg)/2 + 45.0;
  if (bestDeg > 360)
    {
      bestDeg = bestDeg - 360.0;
    }
  printf("\nScale: %f\n",(lScale + rScale)/2);

  printf("leftLoc %d %d\n",(lBestUL.x + lBestLR.x)/2,(lBestUL.y + lBestLR.y)/2);
  printf("rightLoc %d %d\n",(rBestUL.x + rBestLR.x)/2,(rBestUL.y + rBestLR.y)/2);


  printf("\nBest Estimate of Rotation: %f\n",bestDeg);

  //-- Translate scale and position
  float yTrans = 5.8365 - 3.3654*(lScale + rScale)/2;
  float xLeftTrans = ((lBestUL.x + lBestLR.x)/2 - 250)*(4.25/(1100-250));
  float xRightTrans = ((rBestUL.x + rBestLR.x)/2 - 150)*(4.25/(1000-150));

  *yBot = yTrans;
  *xBot = (xLeftTrans+xRightTrans)/2;
  *tBot = bestDeg;


  printf("Estimate of X,Y pos: %f %f",(xLeftTrans+xRightTrans)/2,yTrans);
   
  waitKey(10);
}

void Bot::getTemplateMatch(Mat image, Point* matchUL, Point* matchLR, Mat* templateImage,double* strongestResponse)
{
  //-- This function will take a template image, resize it to the box size, and find it in the image
  //-- This is used as a check on the SURF features update of the box and the edge resize of the box
 
  //-- If the template image doesn't exist, make one
  if(!templateImage->rows)
    {
      checkBounds(image,&upperLeft,&lowerRight);
      *templateImage = image(Range(upperLeft.y,lowerRight.y),Range(upperLeft.x,lowerRight.x));
    }

  Mat result;
  int resultCols =  image.cols - templateImage->cols + 1;
  int resultRows = image.rows - templateImage->rows + 1;
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  
  result.create( resultCols, resultRows, CV_32FC1 );
      
  //-- Resize the template for the current box size
  //Mat resizedTemplate;
  //resize(*templateImage,resizedTemplate,Size( lowerRight.x - upperLeft.x,lowerRight.y - upperLeft.y));
  //-- Match and normalize
  //CV_TIM_SQDIFF
  matchTemplate(image, *templateImage, result, CV_TM_CCOEFF);

  //normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
      
  //-- Grab the best match from the result matrix
  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  //printf("Strongest Match :%f \n",maxVal);
  *matchUL = maxLoc;
    
  //-- Set the other match coordinates
  matchLR->x = matchUL->x + templateImage->cols;
  matchLR->y = matchUL->y + templateImage->rows;
  //-- Normalize response to the area of the template
  *strongestResponse = maxVal/(templateImage->rows*templateImage->cols);
}

void Bot::updateBoxSize(Mat image)
{
  //-- This function uses a sparse set of edges to resize the box describing the car
  //-- This assumes there are many edges on the car, and space between the car and edges in the world
  //-- This will drop small edges on the boundaries, to account for edges like the road that are always near the car

  //-- First determine the size of the bot

  //-- Resize the box

  //-- Match the closest template to get a rotation


  //-- Create an image using Canny edge detection 
  Mat subImage,edges,subEdge;
  
  blur(image, edges, Size(5,5) );
  
  Canny(edges,edges,79,102,3);
  subEdge = edges(Range(upperLeft.y,lowerRight.y),Range(upperLeft.x,lowerRight.x));

  //-- Get the current sum of gradients to compare against
  float currentGradient = sum(subEdge)[0];
  
  //-- This gives us a history of gradient sums to use to check if the current block is similar to the last block
  //-- This did not provide enough information to spend time implementing
  gradientHistory.push_back((int)currentGradient/255);
  double gAvg = std::accumulate(gradientHistory.begin(), gradientHistory.end(), 0) / (double)gradientHistory.size();
  
  //-- Expand the box and see if the increase in captured edges is large enough
  /*
  for (int i=0;i<4;i++)
    {
      Point newUL(upperLeft.x-2*i,upperLeft.y-2*i);
      Point newLR(lowerRight.x+2*i,lowerRight.y+2*i);
      checkBounds(edges,&newUL,&newLR);
      subEdge =  edges(Range(newUL.y,newLR.y),Range(newUL.x,newLR.x));
      //-- If there are enough extra edges, keep the new box size
      if (sum(subEdge)[0]/255 > currentGradient/255 + 25)
	{
	  upperLeft = newUL;
	  lowerRight = newLR;
	}
    }

  //-- The box has been greedily expanded, now contract it until there is more than noise on the edges
  int noiseThreshold = 8;
  while(sum(edges(Range(upperLeft.y,lowerRight.y),Range(upperLeft.x-1,upperLeft.x+1)))[0]/255 < noiseThreshold &&
	upperLeft.x < lowerRight.x - 50)
    {
      upperLeft.x += 1;
    }
 
  while(sum(edges(Range(upperLeft.y-1,upperLeft.y+1),Range(upperLeft.x,lowerRight.x)))[0]/255 < noiseThreshold &&
	upperLeft.y < lowerRight.y - 30)
    {
      upperLeft.y += 1;
    }

  while(sum(edges(Range(upperLeft.y,lowerRight.y),Range(lowerRight.x-1,lowerRight.x+1)))[0]/255 < noiseThreshold &&
	upperLeft.x < lowerRight.x - 50)
    {
      lowerRight.x -= 1;
    }

  while(sum(edges(Range(lowerRight.y-1,lowerRight.y+1),Range(upperLeft.x,lowerRight.x)))[0]/255 < noiseThreshold &&
	upperLeft.y < lowerRight.y - 30) 
    {
      lowerRight.y -= 1;
    }

  */
  checkBounds(edges,&upperLeft,&lowerRight);    
  rectangle(edges,upperLeft,lowerRight,150);  

  Point matchUL, matchLR;    
  double sResponse;
 
  //-- Use the current template image to search for the car in the image
  getTemplateMatch(image,&matchUL,&matchLR,&templateIm, &sResponse);
 
  checkBounds(image, &matchUL, &matchLR);    
 
  rectangle( edges, matchUL, matchLR, 100, 2, 8, 0 );
  
  //-- Allow for error in template matching, up to the overlap factor
  double xOverlap,yOverlap,overlapFactor;
  //-- This will consider success anything more than 50% overlap
  overlapFactor = 0.5;
  xOverlap = (lowerRight.x - upperLeft.x)*overlapFactor;
  yOverlap = (lowerRight.y - upperLeft.y)*overlapFactor;

  if((matchUL.x < upperLeft.x + xOverlap &&
      matchUL.x > upperLeft.x - xOverlap &&
      matchUL.y < upperLeft.y + yOverlap &&
      matchUL.y > upperLeft.y - yOverlap) )
    {
      upperLeft = matchUL;
      lowerRight = matchLR;
      //-- The template matches the current box and it has been 20 frames since we updated the template
      if(updateTemplate==50)
	{
	  templateIm = image(Range(matchUL.y,matchLR.y),Range(matchUL.x,matchLR.x));     updateTemplate = 0;
	  printf("-------------------      Updated template      --------------------\n");
	}
      else updateTemplate++;
      //-- We are tracking the car
      inertialTracking = false;
    }
  else
    {
      //-- The template isn't matching the box
      inertialTracking = true;
    }

  //imshow("Canny Edge Detector",edges);
  waitKey(10);
}

void Bot::checkBounds(Mat image, Point* uL, Point* lR)
{
  //If any point doesn't exist, set it to 0
  if(!uL->x) uL->x = 0;
  if(!uL->y) uL->y = 0;
  if(!lR->x) lR->x = 0;
  if(!lR->y) lR->y = 0;

  //Fix the box points if they are outside of bounds
  //The smallest box is 50,30
  if (uL->x < 1) uL->x = 1;
  if (uL->x > image.size().width) uL->x = image.size().width - 52;
  if (lR->x < uL->x + 50) lR->x = uL->x + 50;  
  if (lR->x > image.size().width) lR->x = image.size().width - 1;
  
    
  if (uL->y < 1) uL->y = 1;
  if (uL->y > image.size().height) uL->y = image.size().height - 32;
  if (lR->y < uL->y + 30) lR->y = uL->y + 30;  
  if (lR->y > image.size().height) lR->y = image.size().height - 1;
  
}


bool Bot::unitTest(Mat image)
{
  bool allTests = true;
  //-- This function is a simple example of unit testing
  //-- There are frameworks that do this better

  Point matchUL,matchLR;
  
  //-- First test checkBounds
  //-- This function is a check used throughout the class
  checkBounds(image, &matchUL, &matchLR);
  if(matchUL.x > 0 && matchUL.x < image.size().width &&
     matchUL.y > 0 && matchUL.y < image.size().height &&
     matchLR.x > 0 && matchLR.x < image.size().width &&
     matchLR.y > 0 && matchLR.y < image.size().height) printf("[ PASS ]");
  else 
    {
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  checkBounds Empty Points\n");

  matchUL.x = 0;
  matchUL.y = 0;
  matchLR.x = 0;
  matchLR.y = 0;

  checkBounds(image, &matchUL, &matchLR);
  if(matchUL.x > 0 && matchUL.x < image.size().width &&
     matchUL.y > 0 && matchUL.y < image.size().height &&
     matchLR.x > 0 && matchLR.x < image.size().width &&
     matchLR.y > 0 && matchLR.y < image.size().height) printf("[ PASS ]");
  else 
    {
      printf("[ FAIL ]"); 
      allTests = false;
    }
  printf("  checkBounds Zero Points\n");

  matchUL.x = image.size().width*2;
  matchUL.y = image.size().height*2;
  matchLR.x = image.size().width*2;
  matchLR.y = image.size().height*2;

  checkBounds(image, &matchUL, &matchLR);
  if(matchUL.x > 0 && matchUL.x < image.size().width &&
     matchUL.y > 0 && matchUL.y < image.size().height &&
     matchLR.x > 0 && matchLR.x < image.size().width &&
     matchLR.y > 0 && matchLR.y < image.size().height) printf("[ PASS ]");
  else 
    {
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  checkBounds Out of Bounds Points\n");

  matchUL.x = image.size().width - 10;
  matchUL.y = image.size().height - 10;
  matchLR.x = 2;
  matchLR.y = 2;

  checkBounds(image, &matchUL, &matchLR);
  if(matchUL.x > 0 && matchUL.x < image.size().width &&
     matchUL.y > 0 && matchUL.y < image.size().height &&
     matchLR.x > 0 && matchLR.x < image.size().width &&
     matchLR.y > 0 && matchLR.y < image.size().height) printf("[ PASS ]");
  else 
    {
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  checkBounds Reversed Points\n");

  if(templateIm.rows) printf("[ PASS ]");
  else 
    {
      printf("[ FAIL ]"); 
      allTests = false; 
    }
  printf("  Constructor Template Doesn't Exist\n");

  //-- Test getTemplateMatch
  Mat emptyTemplate;
  double sResponse;
    getTemplateMatch(image,&matchUL,&matchLR,&emptyTemplate, &sResponse);
  if(matchUL== upperLeft) printf("[ PASS ]");
  else 
    {
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  Empty Template Doesn't Match\n");

  getTemplateMatch(image, &matchUL, &matchLR, &templateIm, &sResponse);
  if(matchUL== upperLeft) printf("[ PASS ]");
  else
    { 
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  Constructor Template Doesn't Match\n");

  //-- Test matchPoints
  std::vector<KeyPoint> i1,i2,o1,o2;
  matchPoints(&i1,&i2,image,image,&o1,&o2,5);
  if(o1.size()) printf("[ PASS ]");
  else
    {
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  matchPoints Empty Keypoints\n");

  if (o1[0].pt.x == o2[0].pt.x && o1[0].pt.y == o2[0].pt.y) printf("[ PASS ]");
  else
    {
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  matchPoints Detector Identical Image\n");
  //-- Test updateBoxPos
  matchUL = upperLeft;
  matchLR = lowerRight;
  //updateBoxPos(image,image,float*, float*, float*);
  if(matchUL.x == upperLeft.x) printf("[ PASS ]");
  else
    {
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  updateBoxPos Identical Image 0 Movement\n");

  //-- Test updateBoxSize - check that the box is about the same size
  matchUL = upperLeft;
  matchLR = lowerRight;
  updateBoxSize(image);
  if(matchUL.x < upperLeft.x + 5 &&
     matchUL.x > upperLeft.x - 5 &&
     matchUL.y < upperLeft.y + 5 &&
     matchUL.y > upperLeft.y - 5 &&
     matchLR.x < lowerRight.x + 5 &&
     matchLR.x > lowerRight.x - 5 &&
     matchLR.x < lowerRight.x + 5 &&
     matchLR.x > lowerRight.x - 5) printf("[ PASS ]");
  else
    {
      printf("[ FAIL ]");
      allTests = false;
    }
  printf("  updateBoxSize Identical Image\n");

  return allTests;
}
