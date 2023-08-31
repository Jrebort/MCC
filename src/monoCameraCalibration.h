#pragma once
#include <string>
#include <opencv2/core.hpp>
#include "Settings.h"
#include "opencv2/objdetect/charuco_detector.hpp"

enum DISPLAY { DETECTION, UNDISTORT};

class monoCameraCalibration
{
private:
	enum {DETECTING = 1, CALIBRATED = 2 };
	Settings s;
	cv::aruco::Dictionary dictionary;
	int mode = DETECTING;
	bool release_object = false;
    clock_t prevTimestamp = 0;
    const char ESC_KEY = 27;

private:	
	double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrors, bool fisheye);

	void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners,
		Settings::Pattern patternType /*= Settings::CHESSBOARD*/);
	
	bool runCalibration(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
		std::vector<float>& reprojErrs, double& totalAvgErr, std::vector<cv::Point3f>& newObjPoints,
		float grid_width, bool release_object);

	void saveCameraParams(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2f> >& imagePoints,
		double totalAvgErr, const std::vector<cv::Point3f>& newObjPoints);
	
	bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		std::vector<std::vector<cv::Point2f>> imagePoints, float grid_width, bool release_object);

	void init();
public:
	const int winSize;
	float grid_width;

    std::vector<std::vector<cv::Point2f> > imagePoints;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Size imageSize;

public:
	monoCameraCalibration(const std::string& settingFilePath, const int& Size);
	~monoCameraCalibration();
	bool calibrate();
	bool showCalibrationResults(DISPLAY displayMode);
};

