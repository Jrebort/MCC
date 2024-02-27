#pragma once
#include <vector>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include "monoCamera.h"

namespace MCC {

	class multiCamera
	{
	public:
		std::string dataPath;
		std::vector<monoCamera> cameraMatrix;
		std::vector<std::vector<cv::Point3d>> worldPoint;
		std::vector<cv::Point3d> worldPointDouble;
		unsigned int cameraNum;

	public:
		multiCamera();
		~multiCamera();
		void addCamera(monoCamera& camera);
		bool initCameraFromData(std::vector<std::string>& viewFolders, double scale);
		bool zhangCalibration(int firstindex, int secondindex);

		void writeCameraParamter();
		bool writeWorldPoint3D();
		bool readWorldPoint3D();
		void MVSTriangluationEval();
		void readCameraParamter();
		void evaluateReprojection();
		void GlobalBA();
		int checkResult();
		void pnpOptimization();
		void sfmCalibration(int firstindex, int secondindex);

	public:
		unsigned int getCameraNum() { return cameraMatrix.size(); }
		unsigned int getPerCameraNum() { return cameraMatrix[1].getImagePointNum(); }
		inline monoCamera& getCamera(unsigned int i) { return cameraMatrix[i]; }
		std::vector<cv::Point3d> getWorldPointVec() const;
		cv::Mat getWorldPointMat() const;
		void visCameraPose();
	};

	// P is return value 
	void computeProjectionMatrix(monoCamera& camera2, cv::Mat P);
} // MCC namespace 
