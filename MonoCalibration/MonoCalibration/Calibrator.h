#pragma once

/*
* Calibrator.h
*  Deals with the Mono/Stereo Camera Calibration
*  Created on: Dec 03, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "MathUtils.h"

#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

class Calibrator
{
public:
	Calibrator();
	~Calibrator();

	// Attributes

	cv::Mat Fun; // Fundamental Matrix
	cv::Mat Ess; // Essential Matrix
	std::vector<std::vector<cv::Point2f>> LeftCorners, RightCorners; // Left and Right Chessboard Corners
	std::vector<std::vector<cv::Point3f>> StereoCornerObjects; // Stereo Corners in Object Space

	// Methods

	/** FUNDAMENTAL MATRIX
	@param Fx_L, Fy_L Focal Length in Pixels for Left Camera.
	@param Fx_R, Fy_R Focal Length in Pixels for Right Camera.
	@param BL Baseline between Stereo Camera Rig (assuming no rotation)
	*/
	void FundamentalMatrix(double Fx_L, double Fy_L, double Fx_R, double Fy_R, double BL);

	/** ESSENTIAL MATRIX
	@param K_L 3x3 Intrinsic Matrix of Left Camera.
	@param K_R 3x3 Intrinsic Matrix of Right Camera.
	@param BL Baseline between Stereo Camera Rig (assuming no rotation)
	*/
	void EssentialMatrix(cv::Mat K_L, cv::Mat K_R, double BL);

	/** FINDS INTRINSIC PARAMETERS OF MONOCULAR CAMERA
	@param img_dir Image directory chessboard calibration images.
	@param img_prefix Image name prefix format.
	@param square_size Chessboard Square Size.
	@param nsquares_height, nsquares_width Number of Chessboard Squares in vertical and horizontal.
	@param K, D Camera matrix and distortion parameters.
	*/
	void IntrinsicParameters(std::string img_dir, std::string img_prefix, int num_imgs, double square_size, int nsquares_height, int nsquares_width, double& Error, cv::Mat& K, cv::Mat& D);

	/** DETECT CHESSBOARD CORNER POINTS WITH STEREO CAMERA
	@param left_dir, right_dir Image directory for left and right chessboard calibration images.
	@param square_size Chessboard Square Size.
	@param nsquares_height, nsquares_width Number of Chessboard Squares in vertical and horizontal.
	*/
	void StereoCornerPoints(std::string left_dir, std::string right_dir, int num_imgs, double square_size, int nsquares_height, int nsquares_width);

	/** PERFORM STEREO CAMERA CALIBRATION
	@param obj_points Corresponding object points.
	@param img_points_L, img_points_R Left and Right Image Points.
	@param K_L, D_L, K_R, D_R Left and Right camera matrix and distortion parameters.
	@param img_size Image Size.
	@param R Matrix between the left and the right camera coordinate systems.
	@param T Vector between the left and the right cameras.
	@param E Essential Matrix.
	@param F Fundamental Matrix.
	*/
	void StereoCalibration(std::vector<std::vector<cv::Point3f>> obj_points, std::vector<std::vector<cv::Point2f>> img_points_L, std::vector<std::vector<cv::Point2f>> img_points_R,
		cv::Mat& K_L, cv::Mat& D_L, cv::Mat& K_R, cv::Mat& D_R, cv::Size img_size, cv::Mat& R, cv::Vec3d& T, cv::Mat& E, cv::Mat& F);

};

#endif /* CALIBRATOR_H_ */