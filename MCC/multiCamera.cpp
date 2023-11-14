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
#include "Dataset.h"
#include "multiCamera.h"
#include "reprojectionError.h"
#include "typeConverter.h"

namespace MCC {
	bool multiCamera::iterateDataFolder(const std::string& foldername, std::vector<std::string>& viewFolders)
	{
		dataPath = foldername;
		using namespace boost::filesystem;
		path p(foldername);
		ASSERT(!(exists(p) && is_directory(p)), "Data folder is not exist! Please check ...");

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
	bool multiCamera::writePoint3D()
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

	bool multiCamera::readPoint3D()
	{
		namespace bfs = boost::filesystem;
		bfs::path filename("CaliPts3D.xml");
		bfs::path dir(dataPath);

		std::string filepath = (dataPath / filename).string();


		cv::FileStorage fs(filepath, cv::FileStorage::READ);
		if (!fs.isOpened())
			return 1;

		fs["Pts"] >> worldPointDouble;

		//using namespace boost::property_tree;
		//ptree pt;

		//read_xml(filepath, pt);

		//// Reading cameraMatrix
		//std::string PtsStr = pt.get<std::string>("opencv_storage.Pts.data");
		//std::stringstream PtsSS(PtsStr);

		//int PointNum = cameraMatrix[1].imagePoints.size() * cameraMatrix[1].imagePoints[0].size();
		//for (int i = 0; i < PointNum; i++)
		//{
		//	int x, y, z;
		//	for(int j = 0; j < 3; j++)
		//	{
		//		PtsSS >> x;
		//		PtsSS >> y;
		//		PtsSS >> z;	
		//		Point3d a(x, y ,z);
		//		worldPointDouble.push_back(a);
		//	}
		//}

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

	void vtoA(std::vector<cv::Point2f> in, cv::Mat& out);
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
		vv2fToV2d(camera1.imagePoints, imagePointVec1);

		cv::Mat inputPtsArray2;
		vv2fToV2d(camera2.imagePoints, imagePointVec2);

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

		for (int i = 2; i < cameraMatrix.size(); i++)
		{
			monoCamera& camera = cameraMatrix[i];
			vector<Point2d>	imagePointVec;
			vv2fToV2d(camera.imagePoints, imagePointVec);

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
				20.0,
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
			for (int j = 0; j < i+1; j++)
			{
				monoCamera& camerainstance = cameraMatrix[j];
				cameras.push_back(&camerainstance);
			}

			std::vector<std::vector<cv::Point2d>> imagePoints;
			for (int j = 0; j < i+1; j++)
			{	
				monoCamera& camerainstance = cameraMatrix[j];
				vector<Point2d> imagePointsVec;
				vv2fToV2d(camerainstance.imagePoints, imagePointsVec);
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
			vv2fToV2d(currentCam.imagePoints, imagePointsVec);
			Mat imgPtsMat = v2dToMat(imagePointsVec);
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
			vv2fToV2d(camerainstance.imagePoints, imagePointsVec);
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
		std::cout << cameraBase.distCoeffs << std::endl;
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
		//window.showWidget("WorldCoordinateSystem", viz::WCoordinateSystem());

		unsigned int camNum = getCameraNum();
		for (size_t i = 0; i < camNum; i++) {
			// 使用Rodrigues变换从旋转向量获得旋转矩阵
			monoCamera& camera = cameraMatrix[i];
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
			viz::WCameraPosition cameraModel(K, 1, viz::Color::white());
			window.showWidget("Cube" + std::to_string(i), cameraModel, pose);

			// 使用箭头表示相机的前向方向
			//Point3d start(0, 0, 0);
			//Point3d end(0, 0, 200);
			//viz::WArrow arrowWidget(start, end, 0.01, viz::Color::red());
			//window.showWidget("Arrow" + std::to_string(i), arrowWidget, pose);

			// 显示编号
			std::string cameraNumber = "C " + std::to_string(i + 1);
			viz::WText3D cameraLabel(cameraNumber, Point3d(0, -0.1, 0), 0.1, false, viz::Color::white());
			window.showWidget("Label" + std::to_string(i), cameraLabel, pose);
		}

		// draw point cloud
		for (int i = 0; i < 88; i++)
		{
			cv::Point3f center;
			center.x = worldPointDouble[i].x;
			center.y = worldPointDouble[i].y;
			center.z = worldPointDouble[i].z;
			viz::WSphere point(center, 0.005, 10, viz::Color::red());
			window.showWidget("point" + std::to_string(i), point);
		}

		for (int i = 88; i < 88+88; i++)
		{
			cv::Point3f center;
			center.x = worldPointDouble[i].x;
			center.y = worldPointDouble[i].y;
			center.z = worldPointDouble[i].z;
			viz::WSphere point(center, 0.005, 10, viz::Color::blue());
			window.showWidget("point" + std::to_string(i), point);
		}

		window.spin();
	}


	void multiCamera::readOptimalResult(const std::string& filename)
	{
		FILE* fptr = fopen(filename.c_str(), "r");

		int num_cameras_;
		int num_points_;
		int num_observations_;
		FscanfOrDie(fptr, "%d", &num_cameras_);
		FscanfOrDie(fptr, "%d", &num_points_);
		FscanfOrDie(fptr, "%d", &num_observations_);


		int* point_index_ = new int[num_observations_];
		int* camera_index_ = new int[num_observations_];
		double* observations_ = new double[2 * num_observations_];

		int num_parameters_ = 15 * num_cameras_ + 3 * num_points_;
		double* parameters_ = new double[num_parameters_];

		for (int i = 0; i < num_observations_; ++i) {
			FscanfOrDie(fptr, "%d", camera_index_ + i);
			FscanfOrDie(fptr, "%d", point_index_ + i);
			for (int j = 0; j < 2; ++j) {
				FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
			}
		}

		for (int i = 0; i < num_parameters_; ++i) {
			FscanfOrDie(fptr, "%lf", parameters_ + i);
		}

		int perCameraParamNum = 15;
		for (int i = 0; i < getCameraNum(); ++i)
		{
			monoCamera& camera = cameraMatrix[i];
			camera.cameraMatrix.at<double>(0, 0) = parameters_[perCameraParamNum * i];
			camera.cameraMatrix.at<double>(1, 1) = parameters_[perCameraParamNum * i + 1];
			camera.cameraMatrix.at<double>(0, 2) = parameters_[perCameraParamNum * i + 2];
			camera.cameraMatrix.at<double>(1, 2) = parameters_[perCameraParamNum * i + 3];

			camera.distCoeffs.at<double>(0, 0) = parameters_[perCameraParamNum * i + 4];
			camera.distCoeffs.at<double>(1, 0) = parameters_[perCameraParamNum * i + 5];
			camera.distCoeffs.at<double>(2, 0) = parameters_[perCameraParamNum * i + 6];
			camera.distCoeffs.at<double>(3, 0) = parameters_[perCameraParamNum * i + 7];
			camera.distCoeffs.at<double>(4, 0) = parameters_[perCameraParamNum * i + 8];

			camera.R.at<double>(0, 0) = parameters_[perCameraParamNum * i + 9];
			camera.R.at<double>(1, 0) = parameters_[perCameraParamNum * i + 10];
			camera.R.at<double>(2, 0) = parameters_[perCameraParamNum * i + 11];

			camera.T.at<double>(0, 0) = parameters_[perCameraParamNum * i + 12];
			camera.T.at<double>(1, 0) = parameters_[perCameraParamNum * i + 13];
			camera.T.at<double>(2, 0) = parameters_[perCameraParamNum * i + 14];
		}
		fclose(fptr);

		delete[] point_index_;
		delete[] camera_index_;
		delete[] observations_;
		delete[] parameters_;
	}

	bool multiCamera::addCameraFromData(std::vector<std::string>& viewFolders)
	{
		for (size_t i = 0; i < viewFolders.size(); i++)
		{
			std::string datasetFolder = viewFolders[i] + "/filter";
			monoCamera camera;
			Dataset dataset(datasetFolder);
			if (!dataset.isCalibrated())
			//if (true)
			{
				std::cout << "Camera " << i + 1 << " is calibrating!" << std::endl;
				dataset.traverseFloder();
				dataset.writeXml();
				dataset.generateSettingXml();
				const std::string filePath = dataset.getSettingPath();
				const int winSize = 11;
				camera.addSettingFilePath(filePath);
				camera.init();
				camera.setScaleFactor(0.25);
				camera.calibrate();
				std::cout << "Camera " << i + 1 << " has finished calibration!" << std::endl;
			}
			else
			{
				std::cout << "Camera " << i + 1 << " is calibrated!" << std::endl;
				const std::string filePath = dataset.getSettingPath();
				const int winSize = 11;
				camera.addSettingFilePath(filePath);
				camera.init();
				camera.readResultXml(dataset.getCameraParamPath());
			}
			addCamera(camera);
		}
		return 0;
	}

	void multiCamera::evaluateReprojection()
	{
		for (int i = 0; i < getCameraNum(); ++i)
		{
			monoCamera& camera = cameraMatrix[i];
			vector<Point2d> imagePointVec;
			vv2fToV2d(camera.imagePoints, imagePointVec);
			double error = computeReprojectionErrors(worldPointDouble,
				imagePointVec,
				camera.R,
				camera.T,
				camera.cameraMatrix,
				camera.distCoeffs,
				false);

			std::cout << "camera" << i+1 << " reprojection error: " << error << " pixel" << std::endl;
		}
	}

	void multiCamera::evaluate()
	{
		for (int i = 0; i < getCameraNum(); ++i)
		{
			monoCamera& camera1 = cameraMatrix[i];	
			for (int j = i; j < getCameraNum(); ++j)
			{
				if (j == i)
					continue;
				monoCamera& camera2 = cameraMatrix[i];	
				computeReprojectionError(camera1, camera2);
			}
		}
	}

	void computeReprojectionError(monoCamera & camera1, monoCamera & camera2)
	{
		vector<Point2f> imagePointVec1;
		vector<Point2f> imagePointVec2;
		vv2fToV2f(camera1.imagePoints, imagePointVec1);
		vv2fToV2f(camera2.imagePoints, imagePointVec2);
		
		vector<Point2f> undistortImagePointVec1;
		vector<Point2f> undistortImagePointVec2;

		// undistort Image Point
		cv::undistortImagePoints(imagePointVec1,
			undistortImagePointVec1,
			camera1.cameraMatrix,
			camera1.distCoeffs);

		cv::undistortImagePoints(imagePointVec2,
			undistortImagePointVec2,
			camera2.cameraMatrix,
			camera2.distCoeffs);

		cv::Mat	P1;
		cv::Mat	P2;
		computeProjectionMatrix(camera1, P1);
		computeProjectionMatrix(camera2, P2);

			
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

	void vtoA(std::vector<cv::Point2f> in, cv::Mat& out)
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
}

