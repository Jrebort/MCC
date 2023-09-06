#pragma once
#include <vector>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include "monoCamera.h"

class multiCamera
{
private:
	std::vector<monoCamera> cameraMatrix;
	std::vector<std::vector<cv::Point3f>> worldPoint;
	unsigned int cameraNum;

public:
	multiCamera();
	~multiCamera();
	void addCamera(monoCamera& camera);
	void writeCameraParamter();
	void pnpOptimization();
	unsigned int getCameraNum() { return getWorldPointVec().size(); }
	unsigned int getPerCameraNum() { return cameraMatrix[1].imagePoints.size(); }
	inline monoCamera& getCamera(unsigned int i) { return cameraMatrix[i]; }
	std::vector<cv::Point3f> getWorldPointVec() const;
	cv::Mat getWorldPointMat() const;
	void visCameraPose();
};