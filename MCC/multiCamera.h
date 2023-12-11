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
		void readOptimalResult(const std::string& filename);
		bool iterateDataFolder(const std::string& DataPath, std::vector<std::string>& viewFolders);
		bool addCameraFromData(std::vector<std::string>& viewFolders);
		void writeCameraParamter();
		bool writePoint3D();
		bool readPoint3D();
		void MVSTriangluationEval();
		void readCameraParamter();
		void evaluateReprojection();
		void GlobalBA();
		void pnpOptimization();
		void sfmCalibration(int firstindex, int secondindex);
		unsigned int getCameraNum() { return cameraMatrix.size(); }
		unsigned int getPerCameraNum() { return cameraMatrix[1].getImagePointNum(); }
		inline monoCamera& getCamera(unsigned int i) { return cameraMatrix[i]; }
		std::vector<cv::Point3d> getWorldPointVec() const;
		cv::Mat getWorldPointMat() const;
		void evaluate();
		void visCameraPose();
	};

	void computeReprojectionError(monoCamera& camera1, monoCamera& camera2);
	void computeProjectionMatrix(monoCamera& camera2, cv::Mat P2);

} // MCC namespace 
