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


int main()
{
	multiCamera multicamera;
	const std::string dataPath = "E:/OneDrive - mails.ucas.edu.cn/Study/Academy/Project/reconstruction/data/database/renkaiwen/right";
	const std::string optimalizationPath = dataPath + "/optimalResult";
	std::vector<std::string> viewFolders;
  
	Step("Check camera need to be calibration");
	addView(viewFolders, dataPath);	

	Step("Calibrate Camera");
	multicamera.addCameraFromData(viewFolders);
	
	Step("Pnp Optimaliztion between two camera");
	multicamera.pnpOptimization();
	//multicamera.writeCameraParamter();
	//multicamera.visCameraPose();
	//multicamera.evaluate();
	Step("Bundle Adjustment Optimaliztion between all camera");
	boost::filesystem::path p(optimalizationPath);
	if (!boost::filesystem::is_regular_file(p))
	{
		Problem multiCCProblem(multicamera); // multi-Camera Calibration Problem	
		BASolver::Solve(multiCCProblem, true, false);
		multiCCProblem.WriteToFile(optimalizationPath);
		multiCCProblem.WriteMultiCamera(multicamera);
	}
	else
	{
		multicamera.readOptimalResult(optimalizationPath);
	}

	multicamera.evaluate();

	multicamera.visCameraPose();
	return 0;
}