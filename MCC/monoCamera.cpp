#include <iostream>
#include "monoCamera.h"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "reprojectionError.h"
#include "progressbar.hpp"

static inline void read(const cv::FileNode& node, Settings& x, const Settings& default_value = Settings())
{
	if (node.empty())
		x = default_value;
	else
		x.read(node);
}

void monoCamera::addSettingFilePath(const std::string& filePath)
{
	settingFilePath = filePath;
}

monoCamera::monoCamera(const std::string& filePath, const int& Size)
	: settingFilePath(settingFilePath), winSize(Size)
{	
}

void monoCamera::init()
{

	cv::FileStorage fs(settingFilePath, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "Could not open the configuration file: \"" 
			<< settingFilePath << "\"" << std::endl;

		CamCaliASSERT(true, "Could not open the configuration file");
	}

	// read setting file 
	fs["Settings"] >> s;
	fs.release();

	// interrupt running if setting format is wrong
	if (!s.goodInput)
	{
		CamCaliASSERT( !s.goodInput ,"Invalid input detected. Application stopping.");
	}

	grid_width = s.squareSize * (s.boardSize.width - 1);
	if (s.calibrationPattern == Settings::Pattern::CHARUCOBOARD) 
	{
		grid_width = s.squareSize * (s.boardSize.width - 2);
	}

	bool release_object = false;	
}

monoCamera::~monoCamera()
{

}

bool monoCamera::cornerDetect()
{
	unsigned short int imgcout = 0;
	used = boost::dynamic_bitset<uint8_t>(s.nrFrames, 0);
	
	progressbar bar(s.nrFrames);
	bar.set_niter(s.nrFrames);
	bar.reset();
	bar.set_todo_char(" ");
	bar.set_done_char("█");
	bar.set_opening_bracket_char("{");
	bar.set_closing_bracket_char("}");
	while ( imgcout < (size_t)s.nrFrames )
	{
		bar.update();
		cv::Mat view; // calibration image
		view = s.nextImage();

		imageSize = view.size();  // Format input image.
		imageSize.width = imageSize.width/scaleFactor;
		imageSize.height = imageSize.height/scaleFactor;

		if (s.flipVertical)    cv::flip(view, view, 0);

		//! [find_pattern]
		std::vector<cv::Point2d> pointBuf;

		bool found; // true if corner found by detector

		int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
		//int chessBoardFlags = cv::CALIB_CB_FAST_CHECK;
		found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);
		//! [find_pattern]

		//! [pattern_found]
		if (found)                // If done with success,
		{
			// improve the found corners' coordinate accuracy for chessboard
			used.set(imgcout);
			cv::Mat viewGray;
			cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
			
			
			std::vector<cv::Point2f> cornerpts(pointBuf.size());
			for (int i = 0; i < pointBuf.size(); i++) {
				cornerpts[i] = cv::Point2f(pointBuf[i].x, pointBuf[i].y);
			}
			cornerSubPix(viewGray, cornerpts, cv::Size(winSize, winSize),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
			for (int i = 0; i < pointBuf.size(); i++) {
				pointBuf[i] = cv::Point2d(cornerpts[i].x, cornerpts[i].y);
			}

			// scale 
			for (auto& point : pointBuf)
			{
				point.x *= 1 / scaleFactor;
				point.y *= 1 / scaleFactor;
			}
		}

		imgcout++;
		if (mode == DETECTING)
		{
			imagePoints.push_back(pointBuf);
		}
	}
	return 1;
}

bool monoCamera::runCalibration(std::vector<float>& reprojErrs, double& totalAvgErr)
{
	std::vector<std::vector<cv::Point3d> > objectPoints(1);

	cv::Size boardSize = s.boardSize;
	float squareSize = s.squareSize;
	for (int i = 0; i < boardSize.height; ++i) {
		for (int j = 0; j < boardSize.width; ++j) {
			objectPoints[0].push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
		}
	}

	objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;

	gridPoints = objectPoints[0];

	objectPoints.resize(imagePoints.size(), objectPoints[0]); // extent to all view

	//Find intrinsic and extrinsic camera parameters
	double rms;

	int iFixedPoint = -1;
	if (release_object)
		iFixedPoint = s.boardSize.width - 1;
	
	std::vector<std::vector<cv::Point3f>> inputobject3d(objectPoints.size());
	for (int i = 0; i < imagePoints.size(); i++)
	{
		inputobject3d[i].reserve(objectPoints[i].size());
		for (int j = 0; j < objectPoints[i].size(); j++) {
			inputobject3d[i].push_back(cv::Point3f(objectPoints[i][j].x, objectPoints[i][j].y, objectPoints[i][j].z));
		}
	}

	std::vector<std::vector<cv::Point2f>> inputimage2d(imagePoints.size());
	for (int i = 0; i < imagePoints.size(); i++)
	{
		inputimage2d[i].reserve(imagePoints[i].size());
		for (int j = 0; j < imagePoints[i].size(); j++) {
			inputimage2d[i].push_back(cv::Point2f(imagePoints[i][j].x, imagePoints[i][j].y));
		}
	}
	rms = calibrateCameraRO(inputobject3d, inputimage2d, imageSize, iFixedPoint,
		cameraMatrix, distCoeffs, rvecs, tvecs, gridPoints,
		s.flag | cv::CALIB_USE_LU | cv::CALIB_FIX_FOCAL_LENGTH |
			 cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_USE_INTRINSIC_GUESS);

	std::cout << "Re-projection error reported by calibrateCamera: " << rms << std::endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	objectPoints.clear();
	objectPoints.resize(imagePoints.size(), gridPoints);
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
		distCoeffs, reprojErrs, s.useFisheye);

	return ok;
}

// Print camera parameters to the output file
void monoCamera::saveCameraParams(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2d> >& imagePoints,
	double totalAvgErr, const std::vector<cv::Point3d>& gridPoints)
{
	cv::FileStorage fs(s.outputFileName, cv::FileStorage::WRITE);

	time_t tm;
	time(&tm);
	struct tm* t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf), "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << s.boardSize.width;
	fs << "board_height" << s.boardSize.height;
	fs << "square_size" << s.squareSize;
	fs << "marker_size" << s.markerSize;

	if (!s.useFisheye && s.flag & cv::CALIB_FIX_ASPECT_RATIO)
		fs << "fix_aspect_ratio" << s.aspectRatio;

	if (s.flag)
	{
		std::stringstream flagsStringStream;
		if (s.useFisheye)
		{
			flagsStringStream << "flags:"
				<< (s.flag & cv::fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
				<< (s.flag & cv::fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
				<< (s.flag & cv::fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
				<< (s.flag & cv::fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
				<< (s.flag & cv::fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
				<< (s.flag & cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
		}
		else
		{
			flagsStringStream << "flags:"
				<< (s.flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
				<< (s.flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
				<< (s.flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
				<< (s.flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
				<< (s.flag & cv::CALIB_FIX_K1 ? " +fix_k1" : "")
				<< (s.flag & cv::CALIB_FIX_K2 ? " +fix_k2" : "")
				<< (s.flag & cv::CALIB_FIX_K3 ? " +fix_k3" : "")
				<< (s.flag & cv::CALIB_FIX_K4 ? " +fix_k4" : "")
				<< (s.flag & cv::CALIB_FIX_K5 ? " +fix_k5" : "");
		}
		fs.writeComment(flagsStringStream.str());
	}

	fs << "flags" << s.flag;

	fs << "fisheye_model" << s.useFisheye;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (s.writeExtrinsics && !reprojErrs.empty())
		fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

	if (s.writeExtrinsics && !rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		cv::Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
		bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
		bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

		for (size_t i = 0; i < rvecs.size(); i++)
		{
			cv::Mat r = bigmat(cv::Range(int(i), int(i + 1)), cv::Range(0, 3));
			cv::Mat t = bigmat(cv::Range(int(i), int(i + 1)), cv::Range(3, 6));

			if (needReshapeR)
				rvecs[i].reshape(1, 1).copyTo(r);
			else
			{
				//*.t() is MatExpr (not Mat) so we can use assignment operator
				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				r = rvecs[i].t();
			}

			if (needReshapeT)
				tvecs[i].reshape(1, 1).copyTo(t);
			else
			{
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				t = tvecs[i].t();
			}
		}
		fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
		fs << "extrinsic_parameters" << bigmat;
	}

	if (s.writePoints && !imagePoints.empty())
	{
		cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (size_t i = 0; i < imagePoints.size(); i++)
		{
			cv::Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
			cv::Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}

	if (s.writeGrid && !gridPoints.empty())
	{
		fs << "grid_points" << gridPoints;
	}
}

//! [run_and_save]
bool monoCamera::runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	std::vector<std::vector<cv::Point2d>> imagePoints, float grid_width, bool release_object)
{
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;
	std::vector<cv::Point3d> gridPoints;

	bool ok = runCalibration(reprojErrs,
		totalAvgErr);
	std::cout << (ok ? "Calibration succeeded" : "Calibration failed")
		<< ". avg re projection error = " << totalAvgErr << std::endl;

	if (ok)
		saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
			totalAvgErr, gridPoints);
	return ok;
}


bool monoCamera::showCalibrationResults(DISPLAY displayMode)
{
	switch (displayMode)
	{
	case DETECTION:
		s.ptrToBeginImage();
		for(auto it = imagePoints.begin(); it != imagePoints.end(); it++)
		{	
			auto pointBuf = (*it);
			cv::Mat view = s.nextImage();

			if (s.calibrationPattern == Settings::CHARUCOBOARD)
				drawChessboardCorners(view, cv::Size(s.boardSize.width - 1, s.boardSize.height - 1), cv::Mat(pointBuf), true);
			else
				drawChessboardCorners(view, s.boardSize, cv::Mat(pointBuf), true);

			const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);
			std::string msg = "reProjection :";
			int baseLine = 0;
			cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
			cv::Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

			msg = cv::format("%d/%d", (int)imagePoints.size(), s.nrFrames);

			putText(view, msg, textOrigin, 1, 1, GREEN);
			cv::imshow("Image View", view);
			while (char key = (char)cv::waitKey(s.delay)) 
			{
				if (key == ESC_KEY) break;
			}
		}

	case UNDISTORT:
		break;
	default:
		break;
	}
	return 0;
}

void monoCamera::writeCornerDetectResultXml(const std::string& xmlFilename)
{
	cv::FileStorage fs(xmlFilename, cv::FileStorage::WRITE);
	if (!fs.isOpened()) {
		std::cerr << "Failed to open " << xmlFilename << std::endl;
		return;
	}
	cv::Mat R = cv::Mat::ones(3, 3, CV_64F);
	cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);

    // write data from the file
	fs << "image_points" << imagePoints;  
	fs << "image_Size" << imageSize;  
	fs << "camera_matrix" << cameraMatrix;  
	fs << "distortion_coefficients" << distCoeffs;
	fs << "R" << R;
	fs << "T" << T;

	// Close the file storage
	fs.release();
}

void monoCamera::readCornerDetectResultXml(const std::string& xmlFilename)
{
	cv::FileStorage fs(xmlFilename, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		std::cerr << "Failed to open " << xmlFilename << std::endl;
		return;
	}

    fs["image_points"] >> imagePoints;  
    fs["image_Size"] >> imageSize;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs["R"] >> R;
    fs["T"] >> T;

	// Close the file storage
	fs.release();
}

std::vector<cv::Point2d> monoCamera::getImagePoint()
{
	using namespace std;
	using namespace cv;
	vector<Point2d> result;
	
	for (auto& patternPoint : imagePoints)
	{
		for (auto& point : patternPoint)
			result.push_back(point);
	}
	return result;
}


bool monoCamera::saveCameraParams(std::string& filename,
	cv::Size& imageSize,
	cv::Mat& cameraMat,
	cv::Mat& distCoeff,
	cv::Mat& r,
	cv::Mat& t)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return 1;

	fs << "imageSize" << imageSize;
	fs << "cameraMatrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;
	fs << "R" << r;
	fs << "T" << t;
	fs.release();
	return 0;
}

bool monoCamera::readCameraParams(std::string& filename)
{
	using namespace boost::property_tree;
	ptree pt;

	read_xml(filename, pt);

	// Reading cameraMatrix
	std::string cameraDataStr = pt.get<std::string>("opencv_storage.cameraMatrix.data");
	std::stringstream camDataSS(cameraDataStr);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			camDataSS >> cameraMatrix.at<double>(i, j);
		}
	}

	// Reading distCoeffs
	std::string distCoeffsDataStr = pt.get<std::string>("opencv_storage.distCoeffs.data");
	std::stringstream distCoeffsSS(distCoeffsDataStr);
	for (int i = 0; i < 5; ++i) {
		distCoeffsSS >> distCoeffs.at<double>(i);
	}

	// Reading rvec
	R = cv::Mat(3, 3, CV_64F);
	std::string rvecDataStr = pt.get<std::string>("opencv_storage.R.data");
	std::stringstream rvecSS(rvecDataStr);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			rvecSS >> R.at<double>(i, j);
		}
	}

	// Reading tvec
	T = cv::Mat(3, 1, CV_64F);
	std::string tvecDataStr = pt.get<std::string>("opencv_storage.T.data");
	std::stringstream tvecSS(tvecDataStr);
	for (int i = 0; i < 3; ++i) {
		tvecSS >> T.at<double>(i);
	}

	//std::cout << cameraMatrix << std::endl;
	return 0;
}

double monoCamera::computeReprojectionErrors(const std::vector<std::vector<cv::Point3d> >& objectPoints,
	const std::vector<std::vector<cv::Point2d> >& imagePoints,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors, bool fisheye)
{
	std::vector<cv::Point2d> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		if (fisheye)
		{
			cv::fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs);
		}
		else
		{
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}
//! [compute_errors]
cv::Mat monoCamera::getProjectMatrix()
{
	cv::Mat P;
	cv::Mat RT;
	cv::hconcat(R, T, RT);
	P = cameraMatrix * RT;

	return P;
}

std::string getFilename(const std::string& filepath) {
	// 查找最后一个目录分隔符
	// 在 Windows 上使用 '\\'，在 UNIX-like 系统上使用 '/'
	size_t pos = filepath.find_last_of("/\\");

	if (pos != std::string::npos) {
		// 如果找到，返回分隔符之后的部分
		return filepath.substr(pos + 1);
	}
	else {
		// 如果没有找到分隔符，整个字符串就是文件名
		return filepath;
	}
}

void monoCamera::visProjImage(std::vector<cv::Point3d>& worldPoints)
{
	std::vector<std::string>& image_files = s.imageList;
	double scale = 1.0;
	int index = 0;

	while (true) {
		cv::Mat resized;
		cv::Mat image = cv::imread(image_files[index]);

		if (image.empty()) {
			std::cerr << "can`t to load image: " << image_files[index] << std::endl;
		}

		// draw point
		auto drawPoint = [&image](double x, double y, cv::Scalar color) {
			cv::Point center(x, y); // 将圆心设置在图像中心
			int radius = 3; // 圆的半径
			cv::circle(image, center, radius, color, 0.5); // 线宽为 2	
			};

		// draw Text

		auto drawText = [&image](double x, double y, std::string s, cv::Scalar color) {
			int fontFace = cv::FONT_HERSHEY_SIMPLEX;
			double fontScale = 0.25;
			int thickness = 1.5;
			int baseline = 0;
			cv::Size textSize = cv::getTextSize(s, fontFace, fontScale, thickness, &baseline);
			cv::Point textOrg(x, y);
			cv::putText(image, s, textOrg, fontFace, fontScale, color, thickness);
			};

		// draw image point
		cv::Scalar color(255, 0, 0); // 绿色
		int imageindex = 0;
		for (auto point : imagePoints[index])
		{
			double x = point.x / 4;
			double y = point.y / 4;
			drawPoint(x, y, color);
			drawText(x, y+5, std::to_string(++imageindex), color);
		}

		// draw projection point
		color = cv::Scalar(0, 0, 255);
		cv::Mat projMatrix = getProjectMatrix();
		cv::Mat homogeneous_point, homogeneous_proj;
		imageindex = 0;
		for (int i = index * 88; i < (index * 88 + 88); i++)
		{
			cv::Point3d point = worldPoints[i];
			homogeneous_point = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1.0);
			homogeneous_proj = projMatrix * homogeneous_point;	
			double x = (homogeneous_proj.at<double>(0)/homogeneous_proj.at<double>(2)) / 4 ;
			double y = (homogeneous_proj.at<double>(1)/homogeneous_proj.at<double>(2)) / 4 ;
			drawPoint(x, y, color);
			drawText(x+4, y, std::to_string(++imageindex), color);
		}
		std::string fn = getFilename(image_files[index]);
		std::string path = R"(Z:\wangxiukia_23_12_06\tmp\)" + fn;
		cv::imwrite(path, image);
		index++;
		if (index >= image_files.size()) exit(0);
	}

	cv::destroyAllWindows();
}

cv::Mat monoCamera::getExtrinsicMatrix()
{
	cv::Mat RT;
	cv::hconcat(R, T, RT);
	return RT;
}
