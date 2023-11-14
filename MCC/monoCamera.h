#pragma once
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include "Settings.h"

enum DISPLAY { DETECTION, UNDISTORT};
enum { DETECTING = 1, CALIBRATED = 2 };
const char ESC_KEY = 27;

class monoCamera
{
private:
	Settings s;
	std::string settingFilePath;
	cv::aruco::Dictionary dictionary;
	int mode = DETECTING;
	float scaleFactor = 4;
	bool release_object = false;

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


public:
	const int winSize;
	float grid_width = 0;
	std::vector<cv::Point3f> gridPoints;
    std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector<cv::Mat> rvecs, tvecs;
	cv::Mat R;
	cv::Mat T;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Size imageSize;

public:
	monoCamera():winSize(11){};
	monoCamera(const std::string& filePath, const int& Size);
	void init();
	void addSettingFilePath(const std::string& settingFilePath);
	~monoCamera();
	float getScaleFactor() { return scaleFactor; }
	std::vector<cv::Point2f> getImagePoint();
	void readResultXml(const std::string& xmlFilename);
	void setScaleFactor(float scale) { scaleFactor = scale; }
	unsigned int getPatternNum() { return imagePoints.size(); }
	inline unsigned int getImagePointNum() { return imagePoints[1].size() * getPatternNum(); }
	bool calibrate();
	bool showCalibrationResults(DISPLAY displayMode);
	bool saveCameraParams(std::string& filename,
							cv::Size& imageSize,
							cv::Mat& cameraMat,
							cv::Mat& distCoeff,
							cv::Mat& r,
							cv::Mat& t);

	bool readCameraParams(std::string& filename);

	cv::Mat getProjectMatrix();
};

