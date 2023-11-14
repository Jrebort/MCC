#pragma once;

#include <vector>
#include <opencv2/core.hpp>

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors, bool fisheye);

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const cv::Mat& rvecs,
	const cv::Mat& tvecs,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors,
	bool fisheye);

double computeReprojectionErrors(const std::vector<cv::Point3d>& objectPoints,
	const std::vector<cv::Point2d>& imagePoints,
	const cv::Mat& rvecs,
	const cv::Mat& tvecs,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	bool fisheye);
