#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "Core.h"
#include "Dataset.h"
#include "monoCamera.h"
#include "multiCamera.h"
#include "lab.h"

int main()
{
	unsigned int STEPNUM = 1;
	MCC::multiCamera multicamera;
	const std::string dataPath = R"(Z:\wangxiukia_23_12_05\right\)";
 
	Step("Check camera need to be calibration");
	std::vector<std::string> viewFolders;
	multicamera.iterateDataFolder(dataPath, viewFolders);
 
	Step("Calibrate Camera");
	multicamera.addCameraFromData(viewFolders);

 	Step("Pnp Optimaliztion between two camera");
	boost::filesystem::path p(dataPath + "/Camera_1.xml");
	if (!boost::filesystem::is_regular_file(p))
	{
		multicamera.sfmCalibration(1,0);
	}
	else
	{
		multicamera.readCameraParamter();
		multicamera.readPoint3D();
	}
	//multicamera.MVSTriangluationEval();
	//multicamera.GlobalBA();
	multicamera.evaluateReprojection();
	//multicamera.visCameraPose();
	multicamera.writeCameraParamter();
	multicamera.writePoint3D();	
	//lab1();

	return 0;
}