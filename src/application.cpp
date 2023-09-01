#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "Dataset.h"
#include "monoCameraCalibration.h"


std::vector<std::string> addView(const std::string& foldername)
{
	using namespace boost::filesystem;
	std::vector<std::string> viewFolders;
	path p(foldername);
	std::cout << "Exist camera folder as follow: " << std::endl;
	// Use boost::filesystem::recursive_directory_iterator for recursive search
	for (directory_iterator it(p); it != directory_iterator(); ++it) {
		if (boost::filesystem::is_directory(it->status())) {
			if (it->path().filename().string().find("view") != std::string::npos) {
				viewFolders.push_back(it->path().string()); 
				std::cout << it->path().string() << std::endl;
			}
		}
	}
	return viewFolders;
}

std::vector<monoCameraCalibration> addCamera(std::vector<std::string> viewFolders)
{
	std::vector<monoCameraCalibration> cameraMatrix;
	for (size_t i = 0; i < viewFolders.size(); i++)
	{
		std::string datasetFolder = viewFolders[i] + "/filter";
		monoCameraCalibration camera;
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
			const std::string filePath = dataset.getSettingPath();
			const int winSize = 11;
			camera.addSettingFilePath(filePath);
			camera.init();
			camera.readResultXml(dataset.getCameraParamPath());
		}
		cameraMatrix.push_back(camera);
	}
	return cameraMatrix;
}

int main()
{
	const std::string dataPath = "E:/OneDrive - mails.ucas.edu.cn/Study/Academy/Project/reconstruction/data/database/yangshuang/right";
	std::vector<std::string> viewFolders = addView(dataPath);
	
	std::vector<monoCameraCalibration> cameraMetrix = addCamera(viewFolders);


	//calibrater.showCalibrationResults(DETECTION);
}