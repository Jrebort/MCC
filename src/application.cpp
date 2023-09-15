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

int main()
{
	unsigned int STEPNUM = 1;
	MCC::multiCamera multicamera;
	const std::string dataPath = "E:/OneDrive - mails.ucas.edu.cn/Study/Academy/Project/reconstruction/data/database/yangshuang/right";
	const std::string optimalizationPath = dataPath + "/optimalResult";
 
	Step("Check camera need to be calibration");
	std::vector<std::string> viewFolders;
	multicamera.iterateDataFolder(dataPath, viewFolders);
 
	Step("Calibrate Camera");
	multicamera.addCameraFromData(viewFolders);

 	Step("Pnp Optimaliztion between two camera");
	multicamera.pnpOptimization();
	//multicamera.visCameraPose();
 
	Step("Bundle Adjustment Optimaliztion between all camera");
	boost::filesystem::path p(optimalizationPath);
	if (!boost::filesystem::is_regular_file(p))
	{
		MCC::Problem multiCCProblem(multicamera); // multi-Camera Calibration Problem	
		MCC::BASolver::Solve(multiCCProblem, true, false);
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