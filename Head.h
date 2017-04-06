#ifndef HEAD_H
#define HEAD_H

#include<time.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

cv::Size boardSize(7,6);
const int BASISL = 10;
const int BASISR = 3383;
const int NUMIMAGES = 22;
const string ROAD = "D:\\SHARE\\Lenovo\\photo\\4";

//camera parameter
const int CAMNUM = 2;
cv::Size RESOLUTION(1280,720);
const int camBRIGHTNESS = 100;	//30-255
const int camEXPOSURE1 = -4;		//-13 -- 0

const int LEFT = 0;
const int RIGHT = 1;

//SGBM parameter
const int numberOfDisparities = 16*4;
const int minDisparity = 16* 6;
#endif