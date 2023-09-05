#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "Dataset.h"
#include "monoCamera.h"
#include "multiCamera.h"


bool addView(std::vector<std::string>& viewFolders, const std::string& foldername)
{
	using namespace boost::filesystem;	
	path p(foldername);
	if (exists(p) && is_directory(p))
		std::cout << "Opening Data folder" << p << std::endl;
	else
		std::cout << "Data folder is not exist! Please check ..." << std::endl;
		__debugbreak();

	std::cout << "Exist camera folder as follow: " << std::endl;
	// Use boost::filesystem::recursive_directory_iterator for recursive search
	for (directory_iterator it(p); it != directory_iterator(); ++it) {
		if (boost::filesystem::is_directory(it->status())) {
			if (it->path().filename().string().find("view") != std::string::npos) {
				viewFolders.push_back(it->path().string()); 
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
	const std::string dataPath = "E:/OneDrive - mails.ucas.edu.cn/Study/Academy/Project/reconstruction/data/database/yangshuang/right";
	std::vector<std::string> viewFolders;

	addView(viewFolders, dataPath);	
	addCamera(multicamera, viewFolders);


	//calibrater.showCalibrationResults(DETECTION);
}