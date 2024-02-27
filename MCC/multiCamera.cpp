#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/sfm/projection.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/sfm/robust.hpp>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "multiCameraBA.h"
#include "Core.h"
#include "Loader.h"
#include "multiCamera.h"
#include "reprojectionError.h"
#include "typeConverter.h"

namespace MCC {
	template<typename T>
	void FscanfOrDie(FILE* fptr, const char* format, T* value) {
		int num_scanned = fscanf(fptr, format, value);
		if (num_scanned != 1)
			std::cerr << "Invalid UW data file. ";
	}

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

	void vtoA(std::vector<cv::Point2d> in, cv::Mat& out);
	bool findCorrectCameraPose(
		const Mat& K,
		const Mat& E,
		const vector<Point2d>& imagePoints1,
		const vector<Point2d>& imagePoints2,
		Mat& outR,
		Mat& outT
	) {
		// 从本质矩阵分解出可能的旋转和平移
		Mat R1, R2, t;
		decomposeEssentialMat(E, R1, R2, t);

		// 生成所有可能的 R 和 t 组合
		vector<Mat> possibleR = { R1, R1, R2, R2 };
		vector<Mat> possiblet = { t, -t, t, -t };

		// 检查每个组合
		for (int i = 0; i < 4; i++) {
			Mat R = possibleR[i];
			Mat T = possiblet[i];

			// 三角测量
			Mat points4D;
			triangulatePoints(K * Mat::eye(3, 4, R.type()), K * (Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0),
				R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1),
				R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2)),
				imagePoints1, imagePoints2, points4D);

			// 转换为欧几里得坐标
			Mat points3D;
			convertPointsFromHomogeneous(points4D.t(), points3D);

			// 正面性检查
			bool allPointsInFront = true;
			for (int j = 0; j < points3D.rows; j++) {
				Vec3d point = points3D.at<Vec3d>(j, 0);
				Mat point_mat = (Mat_<double>(3, 1) << point[0], point[1], point[2]);
				Mat point_in_second_camera_mat = R * point_mat + T;
				Vec3d point_in_second_camera = point_in_second_camera_mat.reshape(1);

				if (point[2] <= 0 || point_in_second_camera[2] <= 0) {
					allPointsInFront = false;
					break;
				}
			}

			// 如果所有点都在两个相机前方，我们找到了正确的姿态
			if (allPointsInFront) {
				outR = R;
				outT = T;
				return true;
			}
		}

		// 如果没有找到有效的姿态，返回 false
		return false;
	}

	double computeReprojectionError(
		const vector<Point2d>& imagePoints1,
		const vector<Point2d>& imagePoints2,
		const Mat& R,
		const Mat& T,
		const Mat& K,
		const Mat& points3D
	) {
		double totalError = 0.0;
		int pointCount = 0;
		Mat projMat1 = K * Mat::eye(3, 4, R.type());
		Mat projMat2 = K * (Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2));

		for (int i = 0; i < points3D.rows; i++) {
			Point3d point3D = Point3d(points3D.at<double>(i, 0),
				points3D.at<double>(i, 1),
				points3D.at<double>(i, 2));

			// Project 3D points back to image plane
			Mat pointMat = (Mat_<double>(4, 1) << point3D.x, point3D.y, point3D.z, 1.0);
			Mat projected1 = projMat1 * pointMat;
			Mat projected2 = projMat2 * pointMat;

			Point2d projectedPt1(
				projected1.at<double>(0, 0) / projected1.at<double>(2, 0),
				projected1.at<double>(1, 0) / projected1.at<double>(2, 0)
			);

			Point2d projectedPt2(
				projected2.at<double>(0, 0) / projected2.at<double>(2, 0),
				projected2.at<double>(1, 0) / projected2.at<double>(2, 0)
			);

			// Calculate the error
			double error1 = norm(imagePoints1[i] - projectedPt1);
			double error2 = norm(imagePoints2[i] - projectedPt2);

			totalError += error1 + error2;
			pointCount += 2;
		}

		// Return the average reprojection error
		return totalError / pointCount;
	}

	std::vector<cv::Point3d> convertMatToPoint3d(const cv::Mat& points3d) {
		std::vector<cv::Point3d> points;

		// 确保points3d是具有3个通道的2D矩阵
		if (points3d.channels() == 3 && points3d.cols == 1) {
			points.reserve(points3d.rows); // 预分配空间以提高性能

			for (int i = 0; i < points3d.rows; ++i) {
				// 使用at<cv::Vec3f>来访问3通道的点
				cv::Vec3d pointVec = points3d.at<cv::Vec3d>(i, 0);
				cv::Point3d point(pointVec[0], pointVec[1], pointVec[2]);
				points.push_back(point);
			}
		}

		return points;
	}

	void computeProjectionMatrix(monoCamera& camera, cv::Mat P)
	{
		cv::Mat R;
		cv::Rodrigues(camera.R, R);
		cv::Mat T = camera.T;
		P = camera.cameraMatrix * (cv::Mat_<double>(3, 4) <<
			R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2));
	}

	void vtoA(std::vector<cv::Point2d> in, cv::Mat& out)
	{
		// 检查输入向量是否为空
		if (in.empty()) {
			out = cv::Mat(); // 如果为空，返回一个空的cv::Mat
			return;
		}

		// 创建一个2行N列的矩阵，N是输入向量的大小
		out = cv::Mat(2, in.size(), CV_32F);

		for (size_t i = 0; i < in.size(); ++i) {
			// 将点的x坐标放入第一行
			out.at<float>(0, i) = in[i].x;
			// 将点的y坐标放入第二行
			out.at<float>(1, i) = in[i].y;
		}
	}



	multiCamera::multiCamera()
	{

	}

	multiCamera::~multiCamera()
	{

	}

	void multiCamera::addCamera(monoCamera& camera)
	{
		cameraMatrix.push_back(camera);
	}

	void multiCamera::writeCameraParamter()
	{
		namespace fs = boost::filesystem;
		for (unsigned int i = 0; i < cameraMatrix.size(); i++) {
			monoCamera& currentCamera = cameraMatrix[i];

			fs::path filename("Camera_" + std::to_string(i + 1) + ".xml");
			fs::path dir(dataPath);

			std::string filepath = (dataPath / filename).string();

			cv::Size& imageSize = currentCamera.imageSize;
			cv::Mat& cameraMat = currentCamera.cameraMatrix;
			cv::Mat& distCoeff = currentCamera.distCoeffs;
			cv::Mat& r = currentCamera.R;
			cv::Mat& t = currentCamera.T;

			currentCamera.saveCameraParams(filepath, imageSize, cameraMat, distCoeff, r, t);
		}

	}
	bool multiCamera::writeWorldPoint3D()
	{
		namespace bfs = boost::filesystem;
		bfs::path filename("CaliPts3D.xml");
		bfs::path dir(dataPath);

		std::string filepath = (dataPath / filename).string();


		cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
		if (!fs.isOpened())
			return 1;

		fs << "Pts" << worldPointDouble;
		fs.release();

		return 0;
	}

	bool multiCamera::readWorldPoint3D()
	{
		namespace bfs = boost::filesystem;
		bfs::path filename("CaliPts3D.xml");
		bfs::path dir(dataPath);

		std::string filepath = (dataPath / filename).string();


		cv::FileStorage fs(filepath, cv::FileStorage::READ);
		if (!fs.isOpened())
			return 1;

		fs["Pts"] >> worldPointDouble;
		return 0;
	}

	void multiCamera::readCameraParamter()
	{
		namespace fs = boost::filesystem;
		for (unsigned int i = 0; i < cameraMatrix.size(); i++) {
			monoCamera& currentCamera = cameraMatrix[i];

			fs::path filename("Camera_" + std::to_string(i + 1) + ".xml");
			fs::path dir(dataPath);

			std::string filepath = (dataPath / filename).string();

			currentCamera.readCameraParams(filepath);
		}

	}

	void multiCamera::sfmCalibration(int firstindex, int secondindex)
	{
		using namespace cv;
		using namespace sfm;
		const unsigned int patternNum = cameraMatrix[0].imagePoints.size();

		for (int i = 0; i < cameraMatrix.size(); i++)
		{
			monoCamera& camera = cameraMatrix[i];
			camera.R = cv::Mat::eye(3, 3, CV_64F);
			camera.T = cv::Mat::zeros(3, 1, CV_64F);
		}

		monoCamera& camera1 = cameraMatrix[firstindex];
		monoCamera& camera2 = cameraMatrix[secondindex];
		vector<Point2d>	imagePointVec1;
		vector<Point2d>	imagePointVec2;

		// generate data format
		cv::Mat inputPtsArray1;
		vv2dToV2d(camera1.imagePoints, imagePointVec1);

		cv::Mat inputPtsArray2;
		vv2dToV2d(camera2.imagePoints, imagePointVec2);

		vector<vector<Point2d>> triangulateInputImagePoints;
		triangulateInputImagePoints.push_back(imagePointVec1);
		triangulateInputImagePoints.push_back(imagePointVec2);

		// 未知内参情况下使用 F 解算 E; 若已知内参K, 应该直接估计E, 准确性更高。
		//cv::Mat F;
		//cv::Mat inliers;
		//float max_error = 1.0;
		//F = cv::findFundamentalMat(imagePointVec1, imagePointVec2, cv::FM_RANSAC, 1, 0.99);

		cv::Mat K;
		K = camera1.cameraMatrix;
		//cv::Mat E = K.t() * F * K;	
		cv::Mat E = findEssentialMat(imagePointVec1, imagePointVec2, K);
		cv::Mat R, T;
		bool foundPose = findCorrectCameraPose(K, E, imagePointVec1, imagePointVec2, R, T);
		camera2.R = R;
		camera2.T = T;

		Mat points4D;
		triangulatePoints(K * Mat::eye(3, 4, R.type()), K * (Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2)),
			imagePointVec1, imagePointVec2, points4D);

		Mat points3D;
		convertPointsFromHomogeneous(points4D.t(), points3D);

		double reprojectionError = computeReprojectionError(imagePointVec1, imagePointVec2, R, T, K, points3D);
		vector<Point3d> worldPoints = convertMatToPoint3d(points3D);

		for (int i = 0; i < cameraMatrix.size(); i++)
		{
			if (i == firstindex && i == secondindex) continue;
			monoCamera& camera = cameraMatrix[i];
			vector<Point2d>	imagePointVec;
			vv2dToV2d(camera.imagePoints, imagePointVec);

			Mat inlier;
			Mat rvec, tvec;
			solvePnPRansac(worldPoints,
				imagePointVec,
				camera.cameraMatrix,
				camera.distCoeffs,
				rvec,
				tvec,
				false,
				10,
				1.0,
				0.99,
				inlier,
				SOLVEPNP_EPNP
			);

			double error = computeReprojectionErrors(worldPoints,
				imagePointVec,
				rvec,
				tvec,
				camera.cameraMatrix,
				camera.distCoeffs,
				false);

			Rodrigues(rvec, camera.R);
			camera.T = tvec;

			// BA
			std::vector<monoCamera*> cameras;
			for (int j = 0; j < i + 1; j++)
			{
				monoCamera& camerainstance = cameraMatrix[j];
				cameras.push_back(&camerainstance);
			}

			std::vector<std::vector<cv::Point2d>> imagePoints;
			for (int j = 0; j < i + 1; j++)
			{
				monoCamera& camerainstance = cameraMatrix[j];
				vector<Point2d> imagePointsVec;
				vv2dToV2d(camerainstance.imagePoints, imagePointsVec);
				imagePoints.push_back(imagePointsVec);
			}

			OptimizeCameraAndPoints(cameras, worldPoints, imagePoints);
			//visCameraPose();
		}

		worldPointDouble = worldPoints;
	}

	void multiCamera::MVSTriangluationEval()
	{
		using namespace cv;
		using namespace std;
		vector<Mat> inputImgPtsVec;
		vector<Mat> ProjVec;
		for (int i = 0; i < cameraMatrix.size(); i++)
		{
			monoCamera& currentCam = cameraMatrix[i];
			vector<Point2d> imagePointsVec;
			vector<Point2d> undistortImgVec;
			vv2dToV2d(currentCam.imagePoints, imagePointsVec);
			cv::undistortImagePoints(imagePointsVec, undistortImgVec, currentCam.cameraMatrix, currentCam.distCoeffs);
			Mat imgPtsMat = v2dToMat(undistortImgVec);
			inputImgPtsVec.push_back(imgPtsMat);

			cv::Mat pose = cv::Mat::zeros(3, 4, CV_64F);
			currentCam.R.copyTo(pose(cv::Rect(0, 0, 3, 3)));
			currentCam.T.copyTo(pose(cv::Rect(3, 0, 1, 3)));

			Mat P = currentCam.cameraMatrix * pose;
			ProjVec.push_back(P);
		}
		Mat point3d;
		sfm::triangulatePoints(inputImgPtsVec, ProjVec, point3d);
		worldPointDouble.clear();
		MatTov3d(point3d, worldPointDouble);
	}


	void multiCamera::GlobalBA()
	{
		std::vector<monoCamera*> cameras;
		for (int i = 0; i < cameraMatrix.size(); i++)
		{
			monoCamera& camerainstance = cameraMatrix[i];
			cameras.push_back(&camerainstance);
		}

		std::vector<std::vector<cv::Point2d>> imagePoints;
		for (int i = 0; i < cameraMatrix.size(); i++)
		{
			monoCamera& camerainstance = cameraMatrix[i];
			vector<Point2d> imagePointsVec;
			vv2dToV2d(camerainstance.imagePoints, imagePointsVec);
			imagePoints.push_back(imagePointsVec);
		}

		OptimizeCameraAndPoints(cameras, worldPointDouble, imagePoints);
	}


	void multiCamera::pnpOptimization()
	{
		using namespace cv;
		using namespace sfm;
		const unsigned int patternNum = cameraMatrix[0].imagePoints.size();
		monoCamera& cameraBase = cameraMatrix[1];

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

			vector<Point3d> worldpoint;
			point3d = r * gridpoint3d + repeat(t, 1, 88);
			mat3dTovector3d(point3d, worldpoint);

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
		vector<Point3d> worldPointVec = getWorldPointVec();
		std::cout << cameraBase.distCoeffs << std::endl;
		for (int i = 0; i < cameraMatrix.size(); i++)
		{
			auto& camera = cameraMatrix[i];
			vector<Point2d> imagePointVec;
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

	std::vector<cv::Point3d> multiCamera::getWorldPointVec() const
	{
		using namespace std;
		unsigned int N = worldPoint.size();
		vector<cv::Point3d> result;

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
		//window.showWidget("WorldCoordinateSystem", viz::WCoordinateSystem());

		unsigned int camNum = getCameraNum();
		for (size_t i = 0; i < camNum; i++) {
		//for (size_t i = 0; i < 2; i++) {
			// 使用Rodrigues变换从旋转向量获得旋转矩阵
			monoCamera& camera = cameraMatrix[i];
			cv::Mat rotation;
			rotation = camera.R;

			// 构建4x4的仿射矩阵
			cv::Mat affine = cv::Mat::zeros(4, 4, CV_64F);
			rotation.copyTo(affine(cv::Rect(0, 0, 3, 3)));
			cv::Mat C = -camera.R.inv() * camera.T;
			C.copyTo(affine(cv::Rect(3, 0, 1, 3)));
			affine.at<double>(3, 3) = 1.0;
			Affine3d pose(affine);

			// 创建一个小立方体来表示相机的体积
			Matx33d K(camera.cameraMatrix);
			//viz::WCameraPosition cameraModel(K, 10, viz::Color::white());
			viz::WCameraPosition cameraModel(40);
			window.showWidget("Cube" + std::to_string(i), cameraModel, pose);

			// 创建棋盘格角点方向射线
			cv::Point2d pt = camera.imagePoints[0][0];
			cv::Mat p_img_hom = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 1);
			cv::Mat p_norm = K.inv() * p_img_hom;
			cv::Mat st = C;
			cv::Mat et = camera.R.inv() * p_norm;
			Point3d startPt(st);
			Point3d endPt(et);
			window.showWidget("Pt" + std::to_string(i), cv::viz::WLine(startPt, startPt + endPt * 450, cv::viz::Color::red()));

			// 使用箭头表示相机的前向方向
			//Point3d start(0, 0, 0);
			//Point3d end(0, 0, 400);
			//viz::WArrow arrowWidget(start, end, 0.001, viz::Color::red());
			//window.showWidget("Arrow" + std::to_string(i), arrowWidget, pose);

			// 显示编号
			std::string cameraNumber = "C " + std::to_string(i + 1);
			viz::WText3D cameraLabel(cameraNumber, Point3d(0, -0.1, 0), 10, false, viz::Color::white());
			window.showWidget("Label" + std::to_string(i), cameraLabel, pose);
		}

		// draw point cloud
		for (int i = 0; i < 88; i++)
		{
			cv::Point3f center;
			center.x = worldPointDouble[i].x;
			center.y = worldPointDouble[i].y;
			center.z = worldPointDouble[i].z;
			viz::WSphere point(center, 1, 10, viz::Color::red());
			window.showWidget("point" + std::to_string(i), point);
		}

		//for (int i = 88; i < 88 + 88; i++)
		//{
		//	cv::Point3f center;
		//	center.x = worldPointDouble[i].x;
		//	center.y = worldPointDouble[i].y;
		//	center.z = worldPointDouble[i].z;
		//	viz::WSphere point(center, 1, 10, viz::Color::blue());
		//	window.showWidget("point" + std::to_string(i), point);
		//}
		cv::Point3d pt1 = worldPointDouble[0];
		cv::Point3d pt2 = worldPointDouble[7];
		cv::Point3d pt3 = worldPointDouble[87];
		cv::Point3d vector1 = pt2 - pt1;
		cv::Point3d vector2 = pt3 - pt1;
		cv::Point3d normal = -vector1.cross(vector2); 
		window.showWidget("Normal", cv::viz::WLine(pt1, pt1 + normal * 3, cv::viz::Color::white()));


		window.spin();
	}

	cv::Mat computeMedianRT(std::vector<cv::Mat>& Tvec) {
		std::vector<double> tx, ty, tz;
		for (const auto& T : Tvec) {
			tx.push_back(T.at<double>(0, 0));
			ty.push_back(T.at<double>(1, 0));
			tz.push_back(T.at<double>(2, 0));
		}

		auto median = [](std::vector<double>& v) {
			size_t n = v.size() / 2;
			std::nth_element(v.begin(), v.begin() + n, v.end());
			return v[n];
			};

		return (cv::Mat_<double>(3, 1) << median(tx), median(ty), median(tz));
	}

	bool stereoCalibration(monoCamera& firstCam, monoCamera& secondCam, cv::Mat& R, cv::Mat& T)
	{	
		cv::Mat E, F;
		cv::Mat perViewError;
		std::vector<std::vector<cv::Point3d> > objectPoints(1);
		cv::Size boardSize = firstCam.s.boardSize;
		float squareSize = firstCam.s.squareSize;
		for (int i = 0; i < boardSize.height; ++i) {
			for (int j = 0; j < boardSize.width; ++j) {
				objectPoints[0].push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
			}
		}

		objectPoints[0][firstCam.s.boardSize.width - 1].x = objectPoints[0][0].x + firstCam.grid_width;


		objectPoints.resize(firstCam.imagePoints.size(), objectPoints[0]); // extent to all view

		//Find intrinsic and extrinsic camera parameters
		double rms;
		int iFixedPoint = -1;

		std::vector<std::vector<cv::Point3f>> inputobject3d(objectPoints.size());
		for (int i = 0; i < firstCam.imagePoints.size(); i++)
		{
			inputobject3d[i].reserve(objectPoints[i].size());
			for (int j = 0; j < objectPoints[i].size(); j++) {
				inputobject3d[i].push_back(cv::Point3f(objectPoints[i][j].x, objectPoints[i][j].y, objectPoints[i][j].z));
			}
		}

		std::vector<std::vector<cv::Point2f>> firstInputimage2d(firstCam.imagePoints.size());
		for (int i = 0; i < firstCam.imagePoints.size(); i++)
		{
			firstInputimage2d[i].reserve(firstCam.imagePoints[i].size());
			for (int j = 0; j < firstCam.imagePoints[i].size(); j++) {
				firstInputimage2d[i].push_back(cv::Point2f(firstCam.imagePoints[i][j].x, firstCam.imagePoints[i][j].y));
			}
		}

		std::vector<std::vector<cv::Point2f>> secondInputimage2d(secondCam.imagePoints.size());
		for (int i = 0; i < secondCam.imagePoints.size(); i++)
		{
			secondInputimage2d[i].reserve(secondCam.imagePoints[i].size());
			for (int j = 0; j < secondCam.imagePoints[i].size(); j++) {
				secondInputimage2d[i].push_back(cv::Point2f(secondCam.imagePoints[i][j].x, secondCam.imagePoints[i][j].y));
			}
		}

		cv::stereoCalibrate(inputobject3d,
			firstInputimage2d,
			secondInputimage2d,
			firstCam.cameraMatrix,
			firstCam.distCoeffs,
			secondCam.cameraMatrix,
			secondCam.distCoeffs,
			firstCam.imageSize,
			R,
			T,
			E,
			F,
			perViewError);
		return 0;
	}

// 	bool multiCamera::zhangCalibration(int firstindex, int secondindex)
// 	{
// 		firstindex -= 1;
// 		secondindex -= 1;
// 
// 		monoCamera& firstCam = cameraMatrix[firstindex];
// 		monoCamera& secondCam = cameraMatrix[secondindex];	
// 		unsigned int patternNum = firstCam.getPatternNum();
// 		cv::Mat R, T;
// 		cv::Mat K;
// 		K = firstCam.cameraMatrix;
// 
// 		double totalAvgErr;
// 		std::vector<float> perViewErr;
// 		for (int i = 0; i < cameraMatrix.size(); i++)
// 		{
// 			monoCamera& camera = cameraMatrix[i];
// 			camera.runCalibration(perViewErr, totalAvgErr);
// 		}
// 			
// 		std::vector<cv::Mat> relativeR, relativeT;
// 		for (int i = 0; i < patternNum; i++)
// 		{
// 			cv::Mat rl, RL, R1, R2, tl, T1, T2, TL;
// 			Rodrigues(firstCam.rvecs[i], R1);
// 			Rodrigues(secondCam.rvecs[i], R2);
// 			T1 = firstCam.tvecs[i];
// 			T2 = secondCam.tvecs[i];
// 			RL = R2 * R1.t();
// 			TL = T2 - R2 * R1.t()* T1;
// 			Rodrigues(RL, rl);
// 			Rodrigues(TL, tl);
// 			relativeR.push_back(rl);
// 			relativeT.push_back(tl);
// 		}
// 		Rodrigues(computeMedianRT(relativeR), secondCam.R);
// 		secondCam.T = computeMedianRT(relativeT);
// 		firstCam.R = cv::Mat::eye(3, 3, CV_64F);
// 		firstCam.T = cv::Mat::zeros(3, 1, CV_64F);
// 
// 		vector<Point2d>	imagePointVec1;
// 		vector<Point2d>	imagePointVec2;
// 
// 		// generate data format
// 		cv::Mat inputPtsArray1;
// 		vv2dToV2d(firstCam.imagePoints, imagePointVec1);
// 
// 		cv::Mat inputPtsArray2;
// 		vv2dToV2d(secondCam.imagePoints, imagePointVec2);
// 
// 		vector<vector<Point2d>> triangulateInputImagePoints;
// 		triangulateInputImagePoints.push_back(imagePointVec1);
// 		triangulateInputImagePoints.push_back(imagePointVec2);
// 
// 		Mat points4D;
// 		triangulatePoints(firstCam.getProjectMatrix(), secondCam.getProjectMatrix(),
// 			imagePointVec1, imagePointVec2, points4D);
// 
// 		Mat points3D;
// 		convertPointsFromHomogeneous(points4D.t(), points3D);
// 
// 		//double reprojectionError = computeReprojectionError(imagePointVec1, imagePointVec2, R, T, K, points3D);
// 		vector<Point3d> worldPoints = convertMatToPoint3d(points3D);
// 
// 		std::vector<monoCamera*> cameras;
// 		cameras.push_back(&firstCam);
// 		cameras.push_back(&secondCam);
// 
// 		std::vector<std::vector<cv::Point2d>> imagePoints;
// 		imagePoints.push_back(imagePointVec1);
// 		imagePoints.push_back(imagePointVec2);
// 
// 		OptimizeCameraAndPoints(cameras, worldPoints, imagePoints);
// 		worldPointDouble = worldPoints;
// 		visCameraPose();
// 
// 		for (int i = 0; i < cameraMatrix.size(); i++)
// 		{
// 			//if (i == firstindex || i == secondindex) continue;
// 			monoCamera& camera = cameraMatrix[i];
// 			vector<Point2d>	imagePointVec;
// 			vv2dToV2d(camera.imagePoints, imagePointVec);
// 
// 			Mat inlier;
// 			Mat rvec, tvec;
// 			solvePnPRansac(worldPoints,
// 				imagePointVec,
// 				camera.cameraMatrix,
// 				camera.distCoeffs,
// 				rvec,
// 				tvec,
// 				false,
// 				100,
// 				0.5,
// 				0.99,
// 				inlier,
// 				SOLVEPNP_EPNP
// 			);
// 
// 			double error = computeReprojectionErrors(worldPoints,
// 				imagePointVec,
// 				rvec,
// 				tvec,
// 				camera.cameraMatrix,
// 				camera.distCoeffs,
// 				false);
// 
// 			Rodrigues(rvec, camera.R);
// 			camera.T = tvec;
// 
// 			// BA
// 			cameras.push_back(&camera);
// 			imagePoints.push_back(imagePointVec);
// 
// 			OptimizeCameraAndPoints(cameras, worldPoints, imagePoints);
// 			std::cout << i << std::endl;
// 			worldPointDouble = worldPoints;
// 			visCameraPose();
// 		}
// 
// 		return 0;
// 	}
// 
 	bool multiCamera::zhangCalibration(int firstindex, int secondindex)
  	{
 		firstindex -= 1;
 		secondindex -= 1;
  		unsigned int patternNum;
  
  		monoCamera& firstCam = cameraMatrix[firstindex];
  		monoCamera& secondCam = cameraMatrix[secondindex];
  		cv::Mat R, T;
  		cv::Mat K;
  		K = firstCam.cameraMatrix;
  
  		// stereo calibration input parameter
		stereoCalibration(firstCam, secondCam, R, T);
   		
 		firstCam.R = cv::Mat::eye(3, 3, CV_64F);
 		firstCam.T = cv::Mat::zeros(3, 1, CV_64F);
 		secondCam.R = R;
  		secondCam.T = T;
  
  		vector<Point2d>	imagePointVec1;
  		vector<Point2d>	imagePointVec2;
  
  		// generate data format
  		cv::Mat inputPtsArray1;
  		vv2dToV2d(firstCam.imagePoints, imagePointVec1);
  
  		cv::Mat inputPtsArray2;
  		vv2dToV2d(secondCam.imagePoints, imagePointVec2);
  
  		vector<vector<Point2d>> triangulateInputImagePoints;
  		triangulateInputImagePoints.push_back(imagePointVec1);
  		triangulateInputImagePoints.push_back(imagePointVec2);
  
  		Mat points4D;
  		triangulatePoints(K * Mat::eye(3, 4, R.type()), K * (Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0),
  			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1),
  			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2)),
  			imagePointVec1, imagePointVec2, points4D);
  
  		Mat points3D;
  		convertPointsFromHomogeneous(points4D.t(), points3D);
  
  		double reprojectionError = computeReprojectionError(imagePointVec1, imagePointVec2, R, T, K, points3D);
  		vector<Point3d> worldPoints = convertMatToPoint3d(points3D);
  
  		std::vector<monoCamera*> cameras;
  		cameras.push_back(&firstCam);
  		cameras.push_back(&secondCam);
 
  		std::vector<std::vector<cv::Point2d>> imagePoints;
 		imagePoints.push_back(imagePointVec1);
 		imagePoints.push_back(imagePointVec2);
 		
  		OptimizeCameraAndPoints(cameras, worldPoints, imagePoints);
  		worldPointDouble = worldPoints;
  		//visCameraPose();
   
  		for (int i = 0; i < cameraMatrix.size(); i++)
  		{
  			if (i == firstindex || i == secondindex) continue;
  			monoCamera& camera = cameraMatrix[i];
  			vector<Point2d>	imagePointVec;
  			vv2dToV2d(camera.imagePoints, imagePointVec);
  
  			Mat inlier;
  			Mat rvec, tvec;
  			solvePnPRansac(worldPoints,
  				imagePointVec,
  				camera.cameraMatrix,
  				camera.distCoeffs,
  				rvec,
  				tvec,
  				false,
  				100,
  				0.5,
  				0.99,
  				inlier,
  				SOLVEPNP_EPNP
  			);
  
  			double error = computeReprojectionErrors(worldPoints,
  				imagePointVec,
  				rvec,
  				tvec,
  				camera.cameraMatrix,
  				camera.distCoeffs,
  				false);
  
  			Rodrigues(rvec, camera.R);
  			camera.T = tvec;
  
  			// BA
  			cameras.push_back(&camera); 
  			imagePoints.push_back(imagePointVec);
  
  			OptimizeCameraAndPoints(cameras, worldPoints, imagePoints);
  			worldPointDouble = worldPoints;
  			//visCameraPose();
  		}
  		visCameraPose();
		
  		return 0;
  	}

  
	int multiCamera::checkResult()
	{
		cv::Point3f pt1, pt2;
		monoCamera& camera = cameraMatrix[0];
		pt1 = worldPointDouble[0];
		pt2 = worldPointDouble[1];
		double dis = cv::norm(pt1-pt2);
		std::cout << "calibration chessboard square error: " << fabs(dis - camera.s.squareSize) << " mm" << std::endl;
		if (fabs(dis - camera.s.squareSize) < 1) return 0;
		else return 1;
	}

	bool multiCamera::initCameraFromData(std::vector<std::string>& viewFolders, double scale)
	{
		unsigned int patternNum = 0;
		for (size_t i = 0; i < viewFolders.size(); i++)
		{
			std::string datasetFolder = viewFolders[i] + "/filter";
			monoCamera camera;
			Loader loader(datasetFolder);
			cv::Mat initCameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
			initCameraMatrix.at<double>(0, 0) = 22758.306779999999;
			initCameraMatrix.at<double>(1, 1) = 22768.670300000002;
			initCameraMatrix.at<double>(0, 2) = 4096;
			initCameraMatrix.at<double>(1, 2) = 2732;
			initCameraMatrix.at<double>(2, 2) = 1;

			cv::Mat initdistMatrix = cv::Mat::zeros(5, 1, CV_64F);

			bool isCalibrated = false;
			if (!loader.isCalibrated())
			{
				std::cout << "Camera " << i + 1 << " is detecting corner point!" << std::endl;
				loader.traverseFloder();
				loader.writeXml();
				loader.generateSettingXml();
				const std::string filePath = loader.getSettingPath();
				camera.addSettingFilePath(filePath);
				camera.init();
				camera.setScaleFactor(scale);
				camera.cornerDetect();
				camera.setCameraMatrix(initCameraMatrix);
				camera.setdistMatrix(initdistMatrix);
				std::cout << "Camera " << i + 1 << " has finished corner detect!" << std::endl;
			}
			else
			{
				std::cout << "Camera " << i + 1 << " is detected corner point!" << std::endl;
				isCalibrated = true;
				const std::string filePath = loader.getSettingPath();
				camera.addSettingFilePath(filePath);
				camera.init();
				camera.readCornerDetectResultXml(loader.getCameraParamPath());

			}
			patternNum = camera.getPatternNum();
			addCamera(camera);

			if ((i == viewFolders.size() - 1) && isCalibrated) return 0;
		}

		// post_processing

		boost::dynamic_bitset<uint8_t> allCamPattern(patternNum);
		allCamPattern.set(); // set 1 to all pos
		for (int i = 0; i < getCameraNum(); i++)
		{
			monoCamera& camera = getCamera(i);
			allCamPattern &= camera.used;
		}
		
		std::cout << allCamPattern << std::endl;

		int sort_dis = 0; // due to erase, affect element index
		for (int i = 0; i < patternNum; i++)
		{
			if (!allCamPattern.test(i))
			{
				for (auto& camera : cameraMatrix)
					camera.imagePoints.erase(camera.imagePoints.begin()+(i-sort_dis));
				sort_dis += 1;
			}
		}

		for (size_t i = 0; i < viewFolders.size(); i++)
		{
			std::string datasetFolder = viewFolders[i] + "/filter";
			monoCamera& camera = getCamera(i);
			Loader loader(datasetFolder);
			camera.writeCornerDetectResultXml(loader.getCameraParamPath());
			//camera.
		}
		
		return 0;
	}

	void multiCamera::evaluateReprojection()
	{
		for (int i = 0; i < getCameraNum(); ++i)
		{
			monoCamera& camera = cameraMatrix[i];
			vector<Point2d> imagePointVec;
			vv2dToV2d(camera.imagePoints, imagePointVec);
			double error = computeReprojectionErrors(worldPointDouble,
				imagePointVec,
				camera.R,
				camera.T,
				camera.cameraMatrix,
				camera.distCoeffs,
				false);

			std::cout << "camera" << i + 1 << " reprojection error: " << error << " pixel" << std::endl;
		}
	}
}
