#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "Core.h"
#include "Loader.h"
#include "monoCamera.h"
#include "multiCamera.h"

int main()
{
	unsigned int STEPNUM = 1;
	MCC::multiCamera multicamera;
	const std::string dataPath = R"(Z:\wangxiukia_23_12_06\calibration\)";
	multicamera.dataPath = dataPath;
 
	Step("Check camera need to be calibration");
	std::vector<std::string> viewFolders;
	MCC::Loader::iterateDataFolder(dataPath, viewFolders);
 
	Step("Calibrate Camera");
	double scale = 0.25;
	multicamera.initCameraFromData(viewFolders, scale);

 	Step("Pnp Optimaliztion between two camera");
	boost::filesystem::path p(dataPath + "/Camera_1.xml");
	//if (!boost::filesystem::is_regular_file(p))
	if(true)
	{
		//multicamera.sfmCalibration(0,3);
		multicamera.zhangCalibration(1,3);
		//multicamera.zhangCalibration(2,5);
	}
	else
	{
		multicamera.readCameraParamter();
		multicamera.readWorldPoint3D();
	}

	multicamera.MVSTriangluationEval();
	multicamera.evaluateReprojection();
	multicamera.GlobalBA();
	multicamera.MVSTriangluationEval();
	multicamera.evaluateReprojection();
	multicamera.checkResult();
	multicamera.visCameraPose();
	multicamera.writeCameraParamter();
	multicamera.writeWorldPoint3D();
	
	for (int i = 0; i < multicamera.getCameraNum(); i++)
	{
		monoCamera& camera = multicamera.getCamera(i);
		std::cout << "Camera " << std::to_string(i) <<
			" Extrinsic Matrix: \n" << camera.getExtrinsicMatrix() << std::endl;
	}


	for (int i = 0; i < multicamera.getCameraNum(); i++)
	{
		monoCamera& camera = multicamera.getCamera(i);
		std::cout << "Camera " << std::to_string(i) <<
			" Projection Matrix: \n" << camera.getProjectMatrix() << std::endl;
	}
	return 0;
}