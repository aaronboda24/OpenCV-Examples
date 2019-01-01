/*
* MathUtils.cpp
* Functions for custom math operations
*
*  Created on: Aug 17, 2018
*      Author: Aaron
*/

#include "pch.h"
#include "MathUtils.h"

using namespace std;
using namespace cv;

// Returns Skew Symmetric Matrix
Mat SkewMat(Vec3d V) {
	Mat Skew = Mat::zeros(3, 3, CV_64F);
	Skew.at<double>(0, 1) = -V(2);	
	Skew.at<double>(0, 2) =  V(1);
	Skew.at<double>(1, 0) =  V(2);
	Skew.at<double>(1, 2) = -V(0);
	Skew.at<double>(2, 0) = -V(1);
	Skew.at<double>(2, 1) =  V(0);
	return Skew;
}