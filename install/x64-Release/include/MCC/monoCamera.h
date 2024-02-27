#pragma once
#include <string>
#include <bitset>
#include <boost/dynamic_bitset.hpp>
#include <opencv2/core.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include "Settings.h"

enum DISPLAY { DETECTION, UNDISTORT};
enum { DETECTING = 1, CALIBRATED = 2 };
const char ESC_KEY = 27;

class monoCamera
{
private:
	std::string settingFilePath;
	cv::aruco::Dictionary dictionary;
	int mode = DETECTING;
	float scaleFactor;
	bool release_object = false;

private:	
	double computeReprojectionErrors(const std::vector<std::vector<cv::Point3d> >& objectPoints,
		const std::vector<std::vector<cv::Point2d> >& imagePoints,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrors, bool fisheye);
	
	void saveCameraParams(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2d> >& imagePoints,
		double totalAvgErr, const std::vector<cv::Point3d>& gridPoints);
	
	bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		std::vector<std::vector<cv::Point2d>> imagePoints, float grid_width, bool release_object);


public:
	Settings s;
	const int winSize;
	float grid_width = 0;
	std::vector<cv::Point3d> gridPoints;
    std::vector<std::vector<cv::Point2d> > imagePoints;
	std::vector<cv::Mat> rvecs, tvecs;	
	boost::dynamic_bitset<uint8_t> used;
	cv::Mat R;
	cv::Mat T;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Size imageSize;

public:
	monoCamera():winSize(11){};
	monoCamera(const std::string& filePath, const int& Size);
	void setCameraMatrix(cv::Mat initmatrix) { cameraMatrix = initmatrix; }
	void setdistMatrix(cv::Mat distmatrix) { distCoeffs = distmatrix; }
	void init();
	void addSettingFilePath(const std::string& settingFilePath);
	~monoCamera();
	float getScaleFactor() { return scaleFactor; }
	std::vector<cv::Point2d> getImagePoint();
	void writeCornerDetectResultXml(const std::string& xmlFilename);
	void readCornerDetectResultXml(const std::string& xmlFilename);
	void visProjImage(std::vector<cv::Point3d>& worldPoints);
	void setScaleFactor(float scale) { scaleFactor = scale; }
	unsigned int getPatternNum() { return imagePoints.size(); }
	inline unsigned int getImagePointNum() { return imagePoints[1].size() * getPatternNum(); }
	bool cornerDetect();
	bool showCalibrationResults(DISPLAY displayMode);
	bool saveCameraParams(std::string& filename,
							cv::Size& imageSize,
							cv::Mat& cameraMat,
							cv::Mat& distCoeff,
							cv::Mat& r,
							cv::Mat& t);

	bool runCalibration(std::vector<float>& reprojErrs, double& totalAvgErr);

	bool readCameraParams(std::string& filename);

	cv::Mat getProjectMatrix();
};

