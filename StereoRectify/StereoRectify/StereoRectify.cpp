// StereoRectify.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Shows how to calibrate and rectify stereo camera images given chessboard calibration stereo image pairs.
// Prepared by Aaron Boda

#include "pch.h"
#include "Calibrator.h"
#include "Rectifier.h"

using namespace cv;
using namespace std;

int main()
{
	// *** Camera Calibration Information
	const int cal_num_imgs = 29, nsquares_height = 9, nsquares_width = 6; // # of internal squares
	const double square_size = 0.02423; // dimension in meters

	string cal_img_dir_L = "Images/Left/";
	string cal_img_dir_R = "Images/Right/";

	// *** Perform Camera Calibration

	// Initialize Calibrator Object
	Calibrator CAL;

	// Result Holders
	double Error_L, Error_R = 0;
	Mat K_L, K_R, D_L, D_R, R, E, F; Vec3d T;

	// Estimate Inrtrinsic Parameters of Individual Cameras
	CAL.IntrinsicParameters(cal_img_dir_L, "left", cal_num_imgs, square_size, nsquares_height, nsquares_width, Error_L, K_L, D_L);
	CAL.IntrinsicParameters(cal_img_dir_R, "right", cal_num_imgs, square_size, nsquares_height, nsquares_width, Error_R, K_R, D_R);

	// Stereo Calibration
	Size img_size = Size(640, 360);
	CAL.StereoCornerPoints(cal_img_dir_L, cal_img_dir_R, cal_num_imgs, square_size, nsquares_height, nsquares_width);
	CAL.StereoCalibration(CAL.StereoCornerObjects, CAL.LeftCorners, CAL.RightCorners, K_L, D_L, K_R, D_R, img_size, R, T, E, F);

	// *** Perform Stereo Rectification

	// Initialize Rectifier Object
	Rectifier REC;

	// Initialize variables
	Mat R_L, R_R, P_L, P_R, Q, K_L_NEW, K_R_NEW, MAPx_L, MAPx_R, MAPy_L, MAPy_R, IMG_L, IMG_R, IMG_L_NEW, IMG_R_NEW;

	// Compute rectification transforms
	REC.StereoRectification(K_L, D_L, K_R, D_R, R, T, R_L, R_R, P_L, P_R, Q, img_size);

	// Compute undistortion and rectification maps for left and right
	REC.UndistortRectifyMap(K_L, D_L, R_L, K_L_NEW, MAPx_L, MAPy_L, img_size);
	REC.UndistortRectifyMap(K_R, D_R, R_R, K_R_NEW, MAPx_R, MAPy_R, img_size);

	// Apply geometrical transformation to image
	string sample_left = cal_img_dir_L + "left1.png";
	string sample_right = cal_img_dir_R + "right1.png";;
	IMG_L = imread(sample_left, IMREAD_GRAYSCALE);
	IMG_R = imread(sample_right, IMREAD_GRAYSCALE);

	REC.Remapper(IMG_L, IMG_L_NEW, MAPx_L, MAPy_L);
	REC.Remapper(IMG_R, IMG_R_NEW, MAPx_R, MAPy_R);

	string img_out_L = "Output/RectifiedLeft.png";
	string img_out_R = "Output/RectifiedRight.png";
	imwrite(img_out_L, IMG_L_NEW);
	imwrite(img_out_R, IMG_R_NEW);

}

