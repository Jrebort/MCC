#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "Core.h"
#include "Dataset.h"
#include "problem.h"
#include "bundleAdjustment.h"
#include "monoCamera.h"
#include "multiCamera.h"


bool addView(std::vector<std::string>& viewFolders, const std::string& foldername)
{
	using namespace boost::filesystem;
	path p(foldername);
	ASSERT(!(exists(p) && is_directory(p)), "Data folder is not exist! Please check ...")

	std::cout << "Opening Data folder: " << p << std::endl;

	std::cout << "Exist camera folder as follow: " << std::endl;
	// Use boost::filesystem::recursive_directory_iterator for recursive search
	for (directory_iterator it(p); it != directory_iterator(); ++it) {
		if (boost::filesystem::is_directory(it->status())) {
			if (it->path().filename().string().find("view") != std::string::npos) {
				viewFolders.push_back(it->path().string()); 
				std::cout << it->path() << std::endl;
			}
		}
	}
	return 0;
}

bool addCamera(multiCamera& multicamera, std::vector<std::string>& viewFolders)
{
	for (size_t i = 0; i < viewFolders.size(); i++)
	{
		std::string datasetFolder = viewFolders[i] + "/filter";
		monoCamera camera;
		Dataset dataset(datasetFolder);
		if (!dataset.isCalibrated())
		{
			dataset.traverseFloder();
			dataset.writeXml();
			dataset.generateSettingXml();
			const std::string filePath = dataset.getSettingPath();
			const int winSize = 11;
			camera.addSettingFilePath(filePath);
			camera.init();
			camera.setScaleFactor(0.25);
			camera.calibrate();
		}
		else
		{	
			std::cout << "Camera "<< i+1 <<" is calibrated!" << std::endl;
			const std::string filePath = dataset.getSettingPath();
			const int winSize = 11;
			camera.addSettingFilePath(filePath);
			camera.init();
			camera.readResultXml(dataset.getCameraParamPath());
		}
		multicamera.addCamera(camera);
	}
	return 0;
}

int main()
{
	multiCamera multicamera;
	const std::string dataPath = "H:/OneDrive - mails.ucas.edu.cn/Study/Academy/Project/reconstruction/data/database/yangshuang/right";
	std::vector<std::string> viewFolders;

	Step("Check camera need to be calibration");
	addView(viewFolders, dataPath);	

	Step("Calibrate Camera");
	addCamera(multicamera, viewFolders);
	
	Step("Pnp Optimaliztion between two camera");
	multicamera.pnpOptimization();
	//multicamera.writeCameraParamter();
	//multicamera.visCameraPose();

	Step("Bundle Adjustment Optimaliztion between all camera");
	Problem multiCCProblem(multicamera); // multi-Camera Calibration Problem
	multiCCProblem.Normalize();
	//multiCCProblem.Perturb(0.1, 0.5, 0.5);
	//multiCCProblem.WriteToPLYFile("initial.ply");

	//BASolver ba(multiCCProblem);
	//ba.Solve();

	//multiCCProblem.WriteToPLYFile("final.ply");
	return 0;
	//calibrater.showCalibrationResults(DETECTION);
}