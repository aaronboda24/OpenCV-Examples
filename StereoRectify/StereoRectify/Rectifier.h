#pragma once
/*
* Rectifier.h
*  Deals with the Stereo Rectification
*  Created on: Dec 05, 2018
*      Author: Aaron Boda
*/

#include "pch.h"

#ifndef RECTIFIER_H_
#define RECTIFIER_H_

// Function

/** PERFORM STEREO CAMERA RECTIFICATION
@param K_L, D_L, K_R, D_R Left and Right camera matrix and distortion parameters.
@param R Rotation Matrix between the left and the right camera coordinate systems.
@param T Translation Vector between the left and the right cameras.
@param R_L, R_R Rectification transform (rotation matrix) for the Left and Right camera..
@param P_L, P_R Projection matrix in the new (rectified) coordinate systems for the Left and Right cameras.
@param Q Disparity-to-depth mapping matrix .
@param img_size Image Size.
*/
void StereoRectification(cv::Mat& K_L, cv::Mat& D_L, cv::Mat& K_R, cv::Mat& D_R, cv::Mat& R, cv::Vec3d& T,
	cv::Mat& R_L, cv::Mat& R_R, cv::Mat& P_L, cv::Mat& P_R, cv::Mat& Q, cv::Size img_size);

/** COMPUTES THE UNDISTORTION AND RECTIFICATION TRANSFORMATION MAP
@param K, D Camera matrix and distortion parameters.
@param R Rectification transform (rotation matrix) for the camera.
@param K_NEW New Camera Matrix.
@param M_X, M_Y Map X and Map Y.
@param img_size Image Size.
*/
void UndistortRectifyMap(cv::Mat& K, cv::Mat& D, cv::Mat& R, cv::Mat& K_NEW, cv::Mat& M_X, cv::Mat& M_Y, cv::Size img_size);

/** REMAP TRANSFORM SOURCE IMAGE USING THE SPECIFIED MAP
@param IMG_IN Input (source) Image.
@param IMG_OUT Output Rectified Image.
@param M_X, M_Y Map X and Map Y.
*/
void Remapper(cv::Mat& IMG_IN, cv::Mat& IMG_OUT, cv::Mat& M_X, cv::Mat& M_Y);



#endif /* RECTIFIER_H_ */