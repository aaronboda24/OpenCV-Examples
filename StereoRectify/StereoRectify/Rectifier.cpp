/*
* Rectifier.cpp
*  Deals with the Stereo Rectification
*  Created on: Dec 05, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "Rectifier.h"

using namespace std;
using namespace cv;

Rectifier::Rectifier()
{
}


Rectifier::~Rectifier()
{
}

// Performs Stereo Camera Calibration
void Rectifier::StereoRectification(Mat& K_L, Mat& D_L, Mat& K_R, Mat& D_R, Mat& R, Vec3d& T, Mat& R_L, Mat& R_R, Mat& P_L, Mat& P_R, Mat& Q, Size img_size) {

	stereoRectify(K_L, D_L, K_R, D_R, img_size, R, T, R_L, R_R, P_L, P_R, Q);

}

// Computes the Undistortion and Rectification Transformation Map
void Rectifier::UndistortRectifyMap(Mat& K, Mat& D, Mat& R, Mat& K_NEW, Mat& M_X, Mat& M_Y, Size img_size) {

	initUndistortRectifyMap(K, D, R, K_NEW, img_size, CV_32F, M_X, M_Y);

}

// Applies a generic geometrical transformation to an image. 
void Rectifier::Remapper(Mat& IMG_IN, Mat& IMG_OUT, Mat& M_X, Mat& M_Y) {

	remap(IMG_IN, IMG_OUT, M_X, M_Y, INTER_LINEAR);

}