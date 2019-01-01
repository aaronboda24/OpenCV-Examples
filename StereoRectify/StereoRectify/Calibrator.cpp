/*
* Calibrator.h
*  Deals with the Mono/Stereo Camera Calibration
*  Created on: Dec 03, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "Calibrator.h"
#include "ImageUtils.h"

using namespace std;
using namespace cv;

Calibrator::Calibrator()
{
	Fun = Mat::zeros(3, 3, CV_64F);
	Ess = Mat::zeros(3, 3, CV_64F);
}

Calibrator::~Calibrator()
{
	Fun.resize(0);
	Ess.resize(0);
}

// Computes Fundamental Matrix and stores it in Class Attribue F
void Calibrator::FundamentalMatrix(double Fx_L, double Fy_L, double Fx_R, double Fy_R, double BL) {
	Vec3d T; T(0) = BL; T(1) = 0; T(2) = 0;
	Mat Tx = Mat::zeros(3, 3, CV_64F); Tx = SkewMat(T);

	Mat K_L = Mat::zeros(3, 3, CV_64F);
	Mat K_R = Mat::zeros(3, 3, CV_64F);
	K_L.at<double>(0, 0) = 1.0 / Fx_L;
	K_L.at<double>(1, 1) = 1.0 / Fy_L;
	K_L.at<double>(2, 2) = 1.0;
	K_R.at<double>(0, 0) = 1.0 / Fx_R;
	K_R.at<double>(1, 1) = 1.0 / Fy_R;
	K_R.at<double>(2, 2) = 1.0;

	Fun = Mat::zeros(3, 3, CV_64F);
	Fun = K_R * Tx * Mat::eye(3, 3, CV_64F) * K_L;
}

// Computes Essential Matrix and stores it in Class Attribue E
void Calibrator::EssentialMatrix(Mat K_L, Mat K_R, double BL) {
	Vec3f T; T(0) = BL; T(1) = 0; T(2) = 0;
	Mat Tx = Mat::zeros(3, 3, CV_64F); Tx = SkewMat(T);

	Ess = Mat::zeros(3, 3, CV_64F);
	Ess = K_R.inv().t() * Tx * Mat::eye(3, 3, CV_64F) * K_L.inv();
}

// Computes intrinsic parameters of camera
void Calibrator::IntrinsicParameters(string img_dir, string img_prefix, int num_imgs, double square_size, int nsquares_height, int nsquares_width, double& Error, Mat& K, Mat& D) {
	Mat img;
	string img_name;
	vector<Point2f> corners;
	vector<vector<Point2f>> points;
	vector<vector<Point3f>> object_points;

	Size board_size = Size(nsquares_width, nsquares_height);

	for (int i = 1; i <= num_imgs; i++) {
		// Update Image File Names
		img_name = img_dir + img_prefix + to_string(i) + ".png";

		// Read Stereo Grayscale Images
		img = imread(img_name, IMREAD_GRAYSCALE);

		// Validate input images
		if (!checkImage(img_name, img))
		{
			cout << " --(!) Error reading image. " << endl;
			continue;
		}

		// Check chessboard corner detection
		bool found = findChessboardCorners(img, board_size, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

		// If detection error, skip image pair
		if (!found)
		{
			cout << "Chessboard corner detection error!" << endl;
			cout << "Image Name: " << img_name << "\n\n";
			continue;
		}

		// Find Chessboard Corner Points
		if (found)
		{
			cornerSubPix(img, corners, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			drawChessboardCorners(img, board_size, corners, found);
		}

		vector<Point3f> obj;
		for (int i = 0; i < nsquares_height; i++)
		{
			for (int j = 0; j < nsquares_width; j++)
			{
				obj.push_back(Point3f((float)j * (float)square_size, (float)i * (float)square_size, 0));
			}
		}

		if (found) {
			//cout << i << "Found corners!" << endl;
			points.push_back(corners);
			object_points.push_back(obj);
		}

		clearImage(img); 
	}

	// Compute Intrinsic Parameters
	vector<Mat> rot_vec, trans_vec;
	vector<Point2f> ImagePoints;
	calibrateCamera(object_points, points, img.size(), K, D, rot_vec, trans_vec, CALIB_FIX_K4 | CALIB_FIX_K5, TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
	
	// Compute Reprojection Errors 
	int total_points = 0;
	double total_error = 0, error = 0;
	vector<double> perViewErrors;
	perViewErrors.resize(object_points.size());

	for (int i = 0; i < (int)object_points.size(); ++i) {
		projectPoints(Mat(object_points[i]), rot_vec[i], trans_vec[i], K, D, ImagePoints);
		int n = (int)object_points[i].size();
		error = norm(Mat(points[i]), Mat(ImagePoints), NORM_L2);
		perViewErrors[i] = sqrt(pow(error,2) / n);
		total_error += pow(error, 2);
		total_points += n;
	}

	Error = sqrt(total_error / total_points);
}

// Detects Corner Points for Stereo Chessboard Images
void Calibrator::StereoCornerPoints(string left_dir, string right_dir, int num_imgs, double square_size, int nsquares_height, int nsquares_width) {
	Mat img_L, img_R;
	string img_name_L, img_name_R;
	vector<Point2f> corners_L, corners_R;
	vector<vector<Point2f>> points_L, points_R;
	vector<vector<Point3f>> corresponding_points;

	Size board_size = Size(nsquares_width, nsquares_height);

	for (int i = 1; i <= num_imgs; i++) {
		// Update Image File Names
		img_name_L = left_dir + "Left" + to_string(i) + ".png";
		img_name_R = right_dir + "Right" + to_string(i) + ".png";
		// Read Stereo Grayscale Images
		img_L = imread(img_name_L, IMREAD_GRAYSCALE);
		img_R = imread(img_name_R, IMREAD_GRAYSCALE);

		// Validate input images
		if (!checkImage(img_name_L, img_L) || !checkImage(img_name_R, img_R))
		{
			cout << " --(!) Error reading stereo images. " << endl;
			continue;
		}

		// Check chessboard corner detection
		bool found_L = findChessboardCorners(img_L, board_size, corners_L, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
		bool found_R = findChessboardCorners(img_R, board_size, corners_R, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

		// If detection error, skip image pair
		if (!found_L || !found_R)
		{
			cout << "Chessboard corner detection error!" << endl;
			cout << "Image Name: " << img_name_L << "\t" << img_name_R << endl;
			continue;
		}

		// Find Stereo Correspondence
		if (found_L)
		{
			cornerSubPix(img_L, corners_L, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			drawChessboardCorners(img_L, board_size, corners_L, found_L);
		}
		if (found_R)
		{
			cornerSubPix(img_R, corners_R, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			drawChessboardCorners(img_R, board_size, corners_R, found_R);
		}

		vector<Point3f> obj;
		for (int i = 0; i < nsquares_height; i++)
		{
			for (int j = 0; j < nsquares_width; j++)
			{
				obj.push_back(Point3f((float)j * (float)square_size, (float)i * (float)square_size, 0));
			}
		}

		if (found_L && found_R) {
			//cout << i << "Found corners!" << endl;
			points_L.push_back(corners_L);
			points_R.push_back(corners_R);
			corresponding_points.push_back(obj);
		}

		clearImage(img_L); clearImage(img_R);
	}

	// Update Class Attributes for Global Use
	LeftCorners = points_L;
	RightCorners = points_R;
	StereoCornerObjects = corresponding_points;
}

// Performs Stereo Camera Calibration
void Calibrator::StereoCalibration(vector<vector<Point3f>> obj_points, vector<vector<Point2f>> img_points_L, vector<vector<Point2f>> img_points_R,
	Mat& K_L, Mat& D_L, Mat& K_R, Mat& D_R, Size img_size, Mat& R, Vec3d& T, Mat& E, Mat& F) {

	double SC = 0;
	SC = stereoCalibrate(obj_points, img_points_L, img_points_R, K_L, D_L, K_R, D_R, img_size, R, T, E, F, 
		CALIB_FIX_INTRINSIC, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));

}
