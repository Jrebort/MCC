#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "Loader.h"
#include "monoCamera.h"
#include "multiCamera.h"

void genMultiCamObject(std::string dataPath, MCC::multiCamera& multicamera, int firstindex, int secondindex)
{
	std::vector<std::string> viewFolders;
	MCC::Loader::iterateDataFolder(dataPath, viewFolders);
	multicamera.dataPath = dataPath;
	double scale = 0.25;
	multicamera.initCameraFromData(viewFolders, scale);
	boost::filesystem::path p(dataPath + "/Camera_1.xml");
	if (!boost::filesystem::is_regular_file(p))
	{
		multicamera.sfmCalibration(firstindex, secondindex);
	}
	else
	{
		multicamera.readCameraParamter();
		multicamera.readWorldPoint3D();
	}
	multicamera.MVSTriangluationEval();
	multicamera.evaluateReprojection();
	//multicamera.visCameraPose();
	//multicamera.GlobalBA();

	//multicamera.writeCameraParamter();
	//multicamera.writeWorldPoint3D();
}

void vizAddmultiCamera(MCC::multiCamera& multicamera, cv::viz::Viz3d window, int id)
{
	using namespace cv;
	unsigned int camNum = multicamera.getCameraNum();

	// set Color
	viz::Color mcolor;
	if (id == 1)
		mcolor = viz::Color::white();
	else
		mcolor = viz::Color::blue();

	for (size_t i = 0; i < camNum; i++) {
		// 使用Rodrigues变换从旋转向量获得旋转矩阵
		monoCamera& camera = multicamera.getCamera(i);
		cv::Mat rotation;
		rotation = camera.R;

		// 构建4x4的仿射矩阵
		cv::Mat affine = cv::Mat::zeros(4, 4, CV_64F);
		rotation.copyTo(affine(cv::Rect(0, 0, 3, 3)));
		camera.T.copyTo(affine(cv::Rect(3, 0, 1, 3)));
		affine.at<double>(3, 3) = 1.0;
		Affine3d pose(affine);

		// 创建一个小立方体来表示相机的体积
		Matx33d K(camera.cameraMatrix);
		viz::WCameraPosition cameraModel(K, 1, mcolor);
		window.showWidget("Cube" + id + std::to_string(i), cameraModel, pose);

		// 显示编号
		std::string cameraNumber = "C " + std::to_string(i + 1);
		viz::WText3D cameraLabel(cameraNumber, Point3d(0, -0.1, 0), 0.1, false, mcolor);
		window.showWidget("Label" + id + std::to_string(i), cameraLabel, pose);
	}
	// draw point cloud
	for (int i = 0; i < 88; i++)
	{
		cv::Point3f center;
		center.x = multicamera.worldPointDouble[i].x;
		center.y = multicamera.worldPointDouble[i].y;
		center.z = multicamera.worldPointDouble[i].z;
		viz::WSphere point(center, 0.003, 10, mcolor);
		window.showWidget("point" + id + std::to_string(i), point);

		std::string cameraNumber = std::to_string(i + 1);
		viz::WText3D cameraLabel(cameraNumber, center, 0.005, false, mcolor);
		window.showWidget("Labelpoint" + id + std::to_string(i), cameraLabel);
	}
	//// draw point cloud
	//for (int i = 88; i < 1000; i++)
	//{
	//	cv::Point3f center;
	//	center.x = multicamera.worldPointDouble[i].x;
	//	center.y = multicamera.worldPointDouble[i].y;
	//	center.z = multicamera.worldPointDouble[i].z;
	//	viz::WSphere point(center, 0.005, 10, mcolor);
	//	window.showWidget("point" + id + std::to_string(i), point);
	//}
}

void viz(MCC::multiCamera& multicamera1, MCC::multiCamera& multicamera2)
{
	// 创建一个viz窗口
	cv::viz::Viz3d window("Camera Poses Visualization");
	vizAddmultiCamera(multicamera1, window, 1);
	vizAddmultiCamera(multicamera2, window, 2);
	window.spin();
}

auto computeRelativePose(cv::Mat R1, cv::Mat T1, cv::Mat R2, cv::Mat T2)
{
	cv::Mat R = R1 * R2.t();
	cv::Mat T = R1 * (-R2.t() * T2) + T1;

	return std::make_pair(R, T);
}

auto camMatrixRelativePose(MCC::multiCamera& multicamera1, MCC::multiCamera& multicamera2)
{
	monoCamera& camera1 = multicamera1.getCamera(1);
	monoCamera& camera2 = multicamera2.getCamera(1);

	cv::Mat R1 = camera1.R;
	cv::Mat R2 = camera2.R;
	cv::Mat T1 = camera1.T;
	cv::Mat T2 = camera2.T;

	auto relativePose = computeRelativePose(R1, T1, R2, T2);

	return relativePose;
}

void convertMultiCamWorldCoord(MCC::multiCamera& multicamera, std::pair<cv::Mat, cv::Mat> pose)
{
	cv::Mat relativeR = pose.first;
	cv::Mat relativeT = pose.second;

	// convert Camera
	for (int i = 0; i < multicamera.getCameraNum(); i++)
	{
		monoCamera& camera = multicamera.getCamera(i);
		camera.R = relativeR * camera.R;
		camera.T = relativeR * camera.T + relativeT;
	}
}

void testRelativePose(MCC::multiCamera& multicamera)
{
	monoCamera& camera1 = multicamera.getCamera(0);
	monoCamera& camera2 = multicamera.getCamera(3);


	cv::Mat R1 = camera1.R;
	cv::Mat R2 = camera2.R;
	cv::Mat T1 = camera1.T;
	cv::Mat T2 = camera2.T;

	auto relativePose = computeRelativePose(R1, T1, R2, T2);
	std::cout << relativePose.first << std::endl;
	std::cout << relativePose.second << std::endl;
}

int main()
{
	MCC::multiCamera multicamera1;
	MCC::multiCamera multicamera2;
	const std::string dataPath1 = R"(Z:\wangxiukia_23_12_06\calibration\)";
	const std::string dataPath2 = R"(Z:\wangxiukia_23_12_06\calibration2\)";
	genMultiCamObject(dataPath1, multicamera1, 2, 1);
	genMultiCamObject(dataPath2, multicamera2, 0, 3);
	std::cout << "************Profile: multicamera1 base view 1 and 0 *************" << std::endl;
	multicamera1.evaluateReprojection();
	std::cout << "************Profile: multicamera2 base view 0 and 1 *************" << std::endl;
	multicamera2.evaluateReprojection();

	//auto pose = camMatrixRelativePose(multicamera2, multicamera1);
	//convertMultiCamWorldCoord(multicamera1, pose);

	//multicamera1.MVSTriangluationEval();
	//multicamera1.GlobalBA();
	//std::cout << "************Profile: multicamera1 base view 1 and 0 *************" << std::endl;
	multicamera1.evaluateReprojection();

	viz(multicamera1, multicamera2);
	return 0;
}
