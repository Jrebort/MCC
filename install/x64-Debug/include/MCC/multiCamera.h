#pragma once
#include <vector>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include "monoCamera.h"

namespace MCC {

	class multiCamera
	{
	private:
		std::string dataPath;
		std::vector<monoCamera> cameraMatrix;
		std::vector<std::vector<cv::Point3d>> worldPoint;
		unsigned int cameraNum;

	public:
		multiCamera();
		~multiCamera();
		void addCamera(monoCamera& camera);
		void readOptimalResult(const std::string& filename);
		bool multiCamera::iterateDataFolder(const std::string& DataPath, std::vector<std::string>& viewFolders);
		bool multiCamera::addCameraFromData(std::vector<std::string>& viewFolders);
		void writeCameraParamter();
		void pnpOptimization();
		unsigned int getCameraNum() { return cameraMatrix.size(); }
		unsigned int getPerCameraNum() { return cameraMatrix[1].getImagePointNum(); }
		inline monoCamera& getCamera(unsigned int i) { return cameraMatrix[i]; }
		std::vector<cv::Point3f> getWorldPointVec() const;
		cv::Mat getWorldPointMat() const;
		void evaluate();
		void visCameraPose();
	};

} // MCC namespace 