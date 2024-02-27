#pragma once 
#include "monoCamera.h"
#include <opencv2/opencv.hpp>
#include <vector>

void OptimizeCameraAndPoints(
	std::vector<monoCamera*>& cameraMatrix,
	std::vector<cv::Point3d>& worldPoints,
	std::vector<std::vector<cv::Point2d>>& imagePoints);
