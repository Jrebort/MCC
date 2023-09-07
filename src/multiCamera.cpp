#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/sfm/projection.hpp>
#include "multiCamera.h"
#include "reprojectionError.h"
#include "typeConverter.h"

cv::Mat averageRotationVectors(const std::vector<cv::Mat>& rotationVectors) {
	cv::Vec4d averageQuat(0, 0, 0, 0);  

	for (const auto& rvec : rotationVectors) {
		cv::Mat rotMat;
		cv::Rodrigues(rvec, rotMat);

		double trace = rotMat.at<double>(0, 0) + rotMat.at<double>(1, 1) + rotMat.at<double>(2, 2);
		double w = std::sqrt(1.0 + trace) / 2.0;
		double x = (rotMat.at<double>(2, 1) - rotMat.at<double>(1, 2)) / (4.0 * w);
		double y = (rotMat.at<double>(0, 2) - rotMat.at<double>(2, 0)) / (4.0 * w);
		double z = (rotMat.at<double>(1, 0) - rotMat.at<double>(0, 1)) / (4.0 * w);

		averageQuat[0] += w;
		averageQuat[1] += x;
		averageQuat[2] += y;
		averageQuat[3] += z;
	}

	averageQuat /= static_cast<double>(rotationVectors.size());

	double len = std::sqrt(averageQuat.dot(averageQuat));
	averageQuat /= len;

	cv::Mat averageRotMat = (cv::Mat_<double>(3, 3) <<
		1 - 2 * (averageQuat[2] * averageQuat[2] + averageQuat[3] * averageQuat[3]),
		2 * (averageQuat[1] * averageQuat[2] - averageQuat[3] * averageQuat[0]),
		2 * (averageQuat[1] * averageQuat[3] + averageQuat[2] * averageQuat[0]),
		2 * (averageQuat[1] * averageQuat[2] + averageQuat[3] * averageQuat[0]),
		1 - 2 * (averageQuat[1] * averageQuat[1] + averageQuat[3] * averageQuat[3]),
		2 * (averageQuat[2] * averageQuat[3] - averageQuat[1] * averageQuat[0]),
		2 * (averageQuat[1] * averageQuat[3] - averageQuat[2] * averageQuat[0]),
		2 * (averageQuat[2] * averageQuat[3] + averageQuat[1] * averageQuat[0]),
		1 - 2 * (averageQuat[1] * averageQuat[1] + averageQuat[2] * averageQuat[2]));

	cv::Mat averageRVec;
	cv::Rodrigues(averageRotMat, averageRVec);

	return averageRVec;
}

multiCamera::multiCamera()
{

}
#include <opencv2/core.hpp>


multiCamera::~multiCamera()
{

}

void multiCamera::addCamera(monoCamera& camera)
{
	cameraMatrix.push_back(camera);
}

void multiCamera::writeCameraParamter()
{

}

void multiCamera::pnpOptimization()
{	
	using namespace cv;
	using namespace sfm;
	const unsigned int patternNum = cameraMatrix[0].imagePoints.size();
	monoCamera& cameraBase = cameraMatrix[0];

	// Generate worldPoint3d based camera0
	for (int i = 0; i < patternNum; i++)
	{
		// set local variable
		Mat r;
		Mat t = cameraBase.tvecs[i].t(); 
		Mat point3d;

		Mat gridpoint3d(cameraBase.gridPoints);
		gridpoint3d.convertTo(gridpoint3d, CV_64F);
		gridpoint3d = gridpoint3d.reshape(1).t();

		Rodrigues(cameraBase.rvecs[i], r, noArray());

		vector<Point3f> worldpoint;
		point3d = r * gridpoint3d + repeat(t, 1, 88);
		mat3dTovector3f(point3d, worldpoint);

		worldPoint.push_back(worldpoint);		
	}

	// modify camera0 rvec and tvec
	Mat unitRotationVector = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
	Mat unitTransitionVector = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
	cameraBase.R = unitRotationVector;
	cameraBase.T = unitTransitionVector;

	std::vector<float> perViewErrors;

	double error = computeReprojectionErrors(worldPoint,
			cameraBase.imagePoints,
			cameraBase.R,
			cameraBase.T,
			cameraBase.cameraMatrix,
			cameraBase.distCoeffs,
			perViewErrors, false);

	assert(error < 1.0); // error exceeds threshold

	// solve PnP between worldPoint and camera ImagePoint
	vector<Point3f> worldPointVec = getWorldPointVec();	
	for (int i = 0; i < cameraMatrix.size(); i++)
	{
		auto& camera = cameraMatrix[i];
		vector<Point2f> imagePointVec;
		Mat imagePointHomo;
		Mat cameraPoint;
		Mat inlier;

		vv2fToV2f(camera.imagePoints, imagePointVec);	

		solvePnPRansac(worldPointVec,
			imagePointVec,
			camera.cameraMatrix,
			camera.distCoeffs,
			camera.R,
			camera.T,
			false,
			10,
			20.0,
			0.99,
			inlier,
			SOLVEPNP_EPNP
		);

		double error = computeReprojectionErrors(worldPointVec,
			imagePointVec,
			camera.R,
			camera.T,
			camera.cameraMatrix,
			camera.distCoeffs,
			false);

		std::cout << "cameraBase and camera" << i + 1 << 
			" Pnp Solve error: mean " << error << " pixel" << std::endl;
	};	
}

 std::vector<cv::Point3f> multiCamera::getWorldPointVec() const
{
	 using namespace std;
	 unsigned int N = worldPoint.size();
	 vector<cv::Point3f> result;
	 
	 for (auto& vec : worldPoint)
	 {
		for (const auto& point : vec)
			result.push_back(point);
	 }

	 return result;
}

cv::Mat multiCamera::getWorldPointMat() const
{
	int N = 0;
	for (const auto& vec : worldPoint) {
		N += vec.size();
	}

	cv::Mat result(3, N, CV_64F);

	int col = 0;
	for (const auto& vec : worldPoint) {
		for (const auto& point : vec) {
			result.at<double>(0, col) = point.x;
			result.at<double>(1, col) = point.y;
			result.at<double>(2, col) = point.z;
			col++;
		}
	}
	return result;
}

void multiCamera::visCameraPose()
{
	// 创建一个viz窗口
	cv::viz::Viz3d window("Camera Poses Visualization");

	// 为世界坐标系创建3D坐标轴，长度为0.5
	window.showWidget("WorldCoordinateSystem", viz::WCoordinateSystem());

	unsigned int camNum = getCameraNum();
	for (size_t i = 0; i < camNum; i++) {
		// 使用Rodrigues变换从旋转向量获得旋转矩阵
		monoCamera& camera = cameraMatrix[i];
		cv::Mat rotation;
		cv::Rodrigues(camera.R, rotation);

		// 构建4x4的仿射矩阵
		cv::Mat affine = cv::Mat::zeros(4, 4, CV_64F);
		rotation.copyTo(affine(cv::Rect(0, 0, 3, 3)));
		camera.T.copyTo(affine(cv::Rect(3, 0, 1, 3)));
		affine.at<double>(3, 3) = 1.0;
		Affine3d pose(affine);

		// 创建一个小立方体来表示相机的体积
		Matx33d K(camera.cameraMatrix);
		viz::WCameraPosition cameraModel(K, 200, viz::Color::white());
		viz::WCube cubeWidget(Point3d(-5, -5, -5), Point3d(5, 5, 5), true, viz::Color::white());
		window.showWidget("Cube" + std::to_string(i), cameraModel, pose);

		// 使用箭头表示相机的前向方向
		Point3d start(0, 0, 0);
		Point3d end(0, 0, 200); 
		viz::WArrow arrowWidget(start, end, 0.01, viz::Color::red());
		window.showWidget("Arrow" + std::to_string(i), arrowWidget, pose);

		// 显示编号
		std::string cameraNumber = "Camera " + std::to_string(i + 1);
		viz::WText3D cameraLabel(cameraNumber, Point3d(0, -0.1, 0), 10, false, viz::Color::white());
		window.showWidget("Label" + std::to_string(i), cameraLabel, pose);
	}

	window.spin();
}

