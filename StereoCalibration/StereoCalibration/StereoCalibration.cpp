// StereoCalibration.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Shows how to use calibrate stereo camera, given chessboard calibration images.
// Prepared by Aaron Boda

#include "pch.h"
#include "Calibrator.h"

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

	cout << "Rotation Matrix: \n";
	cout << R << "\n\n";
	cout << "Translation Vector: \n";
	cout << T << "\n\n";
	cout << "Essential Matrix: \n";
	cout << E << "\n\n";
	cout << "Fundamental Matrix: \n";
	cout << F << "\n\n";

}

