/*
* ImageUtils.h
*  Custom functions for opencv image handling
*  Created on: Dec 03, 2018
*      Author: Aaron Boda
*/

#include "pch.h"

using namespace std;
using namespace cv;

void clearImage(Mat &image_data) {
	image_data = cv::Mat::zeros(image_data.size(), image_data.type());
}

bool checkImage(string image_name, Mat image_data) {
	if (!image_data.data)
	{
		cout << " --(!) Error reading image: " << image_name << endl; 
		return false;
	}
	else
	{
		return true;
	}
}