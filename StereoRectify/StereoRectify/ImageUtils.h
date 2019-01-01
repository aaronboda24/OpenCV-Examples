#pragma once

/*
* ImageUtils.h
*  Custom functions for opencv image handling
*  Created on: Dec 03, 2018
*      Author: Aaron Boda
*/

#ifndef IMAGEUTILS_H_
#define IMAGEUTILS_H_

#include "pch.h"

// Functions
void clearImage(cv::Mat &image_data);
bool checkImage(std::string image_name, cv::Mat image_data);

#endif /* IMAGEUTILS_H_ */
