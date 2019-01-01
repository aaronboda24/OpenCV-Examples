// MonoCalibration.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Shows how to use OpenCV to find intrinsic parameters of a camera, given chessboard calibration images.
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

	string cal_img_dir = "Images/";

	// *** Perform Camera Calibration
	
	// Initialize Calibrator Object
	Calibrator CAL;

	// Result Holders
	double Error = 0; Mat K, D;

	// Estimate Inrtrinsic Parameters
	CAL.IntrinsicParameters(cal_img_dir, "img", cal_num_imgs, square_size, nsquares_height, nsquares_width, Error, K, D);

	cout << "CAMERA INTRINSIC PARAMETERS: " << "\n";
	cout << "Reprojection Error: " << Error << endl;
	cout << "Camera Matrix: \n" << K << endl;
	cout << "Distortion Parameters: \n" << D << endl;

}

