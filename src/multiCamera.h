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
	cv::Mat worldPoint;

public:
	multiCamera();
	~multiCamera();
	void addCamera(monoCamera& camera);
	void writeCameraParamter();
	void pnpOptimization();
	void visCameraPose();
};