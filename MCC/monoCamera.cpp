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
	
	//create CharucoBoard
	if (s.calibrationPattern == Settings::CHARUCOBOARD) 
	{
		if (s.arucoDictFileName == "") {
			cv::aruco::PredefinedDictionaryType arucoDict;
			if (s.arucoDictName == "DICT_4X4_50") { arucoDict = cv::aruco::DICT_4X4_50; }
			else if (s.arucoDictName == "DICT_4X4_100") { arucoDict = cv::aruco::DICT_4X4_100; }
			else if (s.arucoDictName == "DICT_4X4_250") { arucoDict = cv::aruco::DICT_4X4_250; }
			else if (s.arucoDictName == "DICT_4X4_1000") { arucoDict = cv::aruco::DICT_4X4_1000; }
			else if (s.arucoDictName == "DICT_5X5_50") { arucoDict = cv::aruco::DICT_5X5_50; }
			else if (s.arucoDictName == "DICT_5X5_100") { arucoDict = cv::aruco::DICT_5X5_100; }
			else if (s.arucoDictName == "DICT_5X5_250") { arucoDict = cv::aruco::DICT_5X5_250; }
			else if (s.arucoDictName == "DICT_5X5_1000") { arucoDict = cv::aruco::DICT_5X5_1000; }
			else if (s.arucoDictName == "DICT_6X6_50") { arucoDict = cv::aruco::DICT_6X6_50; }
			else if (s.arucoDictName == "DICT_6X6_100") { arucoDict = cv::aruco::DICT_6X6_100; }
			else if (s.arucoDictName == "DICT_6X6_250") { arucoDict = cv::aruco::DICT_6X6_250; }
			else if (s.arucoDictName == "DICT_6X6_1000") { arucoDict = cv::aruco::DICT_6X6_1000; }
			else if (s.arucoDictName == "DICT_7X7_50") { arucoDict = cv::aruco::DICT_7X7_50; }
			else if (s.arucoDictName == "DICT_7X7_100") { arucoDict = cv::aruco::DICT_7X7_100; }
			else if (s.arucoDictName == "DICT_7X7_250") { arucoDict = cv::aruco::DICT_7X7_250; }
			else if (s.arucoDictName == "DICT_7X7_1000") { arucoDict = cv::aruco::DICT_7X7_1000; }
			else if (s.arucoDictName == "DICT_ARUCO_ORIGINAL") { arucoDict = cv::aruco::DICT_ARUCO_ORIGINAL; }
			else if (s.arucoDictName == "DICT_APRILTAG_16h5") { arucoDict = cv::aruco::DICT_APRILTAG_16h5; }
			else if (s.arucoDictName == "DICT_APRILTAG_25h9") { arucoDict = cv::aruco::DICT_APRILTAG_25h9; }
			else if (s.arucoDictName == "DICT_APRILTAG_36h10") { arucoDict = cv::aruco::DICT_APRILTAG_36h10; }
			else if (s.arucoDictName == "DICT_APRILTAG_36h11") { arucoDict = cv::aruco::DICT_APRILTAG_36h11; }
			else {
				std::cout << "incorrect name of aruco dictionary \n";
			}

			dictionary = cv::aruco::getPredefinedDictionary(arucoDict);
		}
		else {
			cv::FileStorage dict_file(s.arucoDictFileName, cv::FileStorage::Mode::READ);
			cv::FileNode fn(dict_file.root());
			dictionary.readDictionary(fn);

			cv::aruco::CharucoBoard ch_board({s.boardSize.width, s.boardSize.height}, s.squareSize, s.markerSize, dictionary);
			cv::aruco::CharucoDetector ch_detector(ch_board);
		}
	}
	else {
		// default dictionary
		dictionary = cv::aruco::getPredefinedDictionary(0);
	}
}

monoCamera::~monoCamera()
{

}

bool monoCamera::calibrate()
{
	while ( imagePoints.size() < (size_t)s.nrFrames )
	{
		std::cout << imagePoints.size() << std::endl;
		cv::Mat view; // calibration image
		view = s.nextImage();

		imageSize = view.size();  // Format input image.
		imageSize.width = imageSize.width/scaleFactor;
		imageSize.height = imageSize.height/scaleFactor;

		if (s.flipVertical)    cv::flip(view, view, 0);

		//! [find_pattern]
		std::vector<cv::Point2f> pointBuf;

		bool found; // true if corner found by detector

		int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;

		if (!s.useFisheye) {
			// fast check erroneously fails with high distortions like fisheye
			chessBoardFlags |= cv::CALIB_CB_FAST_CHECK;
		}

		switch (s.calibrationPattern) // Find feature points on the input format
		{
			case Settings::CHESSBOARD:
				found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);
				break;
				//[TODO]
				//case Settings::CHARUCOBOARD:
				//    ch_detector.detectBoard( view, pointBuf, markerIds);
				//    found = pointBuf.size() == (size_t)((s.boardSize.height - 1)*(s.boardSize.width - 1));
				//    break;
			case Settings::CIRCLES_GRID:
				found = findCirclesGrid(view, s.boardSize, pointBuf);
				break;
			case Settings::ASYMMETRIC_CIRCLES_GRID:
				found = findCirclesGrid(view, s.boardSize, pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
				break;
			default:
				found = false;
				break;
		}
		//! [find_pattern]

		//! [pattern_found]
		if (found)                // If done with success,
		{
			// improve the found corners' coordinate accuracy for chessboard
			if (s.calibrationPattern == Settings::CHESSBOARD)
			{
				cv::Mat viewGray;
				cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
				cornerSubPix(viewGray, pointBuf, cv::Size(winSize, winSize),
					cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
			}

			// scale 
			for (auto& point : pointBuf)
			{
				point.x *= 1 / scaleFactor;
				point.y *= 1 / scaleFactor;
			}


			if (mode == DETECTING)
			{
				imagePoints.push_back(pointBuf);
			}

			//------------------------- Video capture  output  undistorted ------------------------------
			//! [output_undistorted]
			if (mode == CALIBRATED && s.showUndistorted)
			{
				cv::Mat temp = view.clone();
				if (s.useFisheye)
				{
					cv::Mat newCamMat;
					cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
						cv::Matx33d::eye(), newCamMat, 1);
					cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs, newCamMat);
				}
				else
					undistort(temp, view, cameraMatrix, distCoeffs);
			}
			//! [output_undistorted]
			//------------------------------ Show image and check for input commands -------------------
			//! [await_input]
		}
		else
		{
			//TODO: register all image
			__debugbreak;
		}

		// -----------------------Show the undistorted image for the image list ------------------------
		//! [show_results]
		if (s.showUndistorted && !cameraMatrix.empty())
		{
			cv::Mat view, rview, map1, map2;

			if (s.useFisheye)
			{
				cv::Mat newCamMat;
				cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
					cv::Matx33d::eye(), newCamMat, 1);
				cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Matx33d::eye(), newCamMat, imageSize,
					CV_16SC2, map1, map2);
			}
			else
			{
				initUndistortRectifyMap(
					cameraMatrix, distCoeffs, cv::Mat(),
					getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
					CV_16SC2, map1, map2);
			}

			for (size_t i = 0; i < s.imageList.size(); i++)
			{
				view = cv::imread(s.imageList[i], cv::IMREAD_COLOR);
				if (view.empty())
					continue;
				remap(view, rview, map1, map2, cv::INTER_LINEAR);
				imshow("Image View", rview);
				while (char c = (char)cv::waitKey())
				{
					if (c == ESC_KEY || c == 'q' || c == 'Q')
						break;
				}
			}
		}
		//! [show_results]
	}

	if (mode == DETECTING && imagePoints.size() >= (size_t)s.nrFrames)
	{
		if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, grid_width,
			release_object))
		{
			mode = CALIBRATED;
			return 0;
		}
	}
	return 1;
}

//! [compute_errors]
double monoCamera::computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors, bool fisheye)
{
	std::vector<cv::Point2f> imagePoints2;
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

//! [board_corners]
void monoCamera::calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners,
	Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
	corners.clear();

	switch (patternType)
	{
	case Settings::CHESSBOARD:
	case Settings::CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; ++i) {
			for (int j = 0; j < boardSize.width; ++j) {
				corners.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
			}
		}
		break;
	case Settings::CHARUCOBOARD:
		for (int i = 0; i < boardSize.height - 1; ++i) {
			for (int j = 0; j < boardSize.width - 1; ++j) {
				corners.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
			}
		}
		break;
	case Settings::ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++) {
			for (int j = 0; j < boardSize.width; j++) {
				corners.push_back(cv::Point3f((2 * j + i % 2) * squareSize, i * squareSize, 0));
			}
		}
		break;
	default:
		break;
	}
}
//! [board_corners]

bool monoCamera::runCalibration(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
	std::vector<float>& reprojErrs, double& totalAvgErr, std::vector<cv::Point3f>& newObjPoints,
	float grid_width, bool release_object)
{
	//! [fixed_aspect]
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = (double)22758.30678; // Set f_x
	cameraMatrix.at<double>(1, 1) = (double)22768.67030; // Set f_y
	cameraMatrix.at<double>(0, 2) = (double)4096.0; // Set c_x
	cameraMatrix.at<double>(1, 2) = (double)2732.0; // Set c_y


	//if (!s.useFisheye && s.flag & cv::CALIB_FIX_ASPECT_RATIO)
	//	cameraMatrix.at<double>(0, 0) = s.aspectRatio;
	//! [fixed_aspect]
	if (s.useFisheye) {
		distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
	}
	else {
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	}

	std::vector<std::vector<cv::Point3f> > objectPoints(1);
	calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
	if (s.calibrationPattern == Settings::Pattern::CHARUCOBOARD) {
		objectPoints[0][s.boardSize.width - 2].x = objectPoints[0][0].x + grid_width;
	}
	else {
		objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
	}
	newObjPoints = objectPoints[0];

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms;

	if (s.useFisheye) {
		cv::Mat _rvecs, _tvecs;
		rms = cv::fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
			_tvecs, s.flag);

		rvecs.reserve(_rvecs.rows);
		tvecs.reserve(_tvecs.rows);
		for (int i = 0; i < int(objectPoints.size()); i++) {
			rvecs.push_back(_rvecs.row(i));
			tvecs.push_back(_tvecs.row(i));
		}
	}
	else {
		int iFixedPoint = -1;
		if (release_object)
			iFixedPoint = s.boardSize.width - 1;
		std::cout << cameraMatrix << std::endl;
		rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
			cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
			s.flag | cv::CALIB_USE_LU | cv::CALIB_FIX_FOCAL_LENGTH |
				 cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_USE_INTRINSIC_GUESS);
		std::cout << cameraMatrix << std::endl;
	}

	if (release_object) {
		std::cout << "New board corners: " << std::endl;
		std::cout << newObjPoints[0] << std::endl;
		std::cout << newObjPoints[s.boardSize.width - 1] << std::endl;
		std::cout << newObjPoints[s.boardSize.width * (s.boardSize.height - 1)] << std::endl;
		std::cout << newObjPoints.back() << std::endl;
	}

	std::cout << "Re-projection error reported by calibrateCamera: " << rms << std::endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	objectPoints.clear();
	objectPoints.resize(imagePoints.size(), newObjPoints);
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
		distCoeffs, reprojErrs, s.useFisheye);

	return ok;
}

// Print camera parameters to the output file
void monoCamera::saveCameraParams(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2f> >& imagePoints,
	double totalAvgErr, const std::vector<cv::Point3f>& newObjPoints)
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

	if (s.writeGrid && !newObjPoints.empty())
	{
		fs << "grid_points" << newObjPoints;
		gridPoints = newObjPoints;
	}
}

//! [run_and_save]
bool monoCamera::runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	std::vector<std::vector<cv::Point2f>> imagePoints, float grid_width, bool release_object)
{
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;
	std::vector<cv::Point3f> newObjPoints;

	bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
		totalAvgErr, newObjPoints, grid_width, release_object);
	std::cout << (ok ? "Calibration succeeded" : "Calibration failed")
		<< ". avg re projection error = " << totalAvgErr << std::endl;

	if (ok)
		saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
			totalAvgErr, newObjPoints);
	return ok;
}
//! [run_and_save]

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

void monoCamera::readResultXml(const std::string& xmlFilename)
{
	cv::FileStorage fs(xmlFilename, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		std::cerr << "Failed to open " << xmlFilename << std::endl;
		return;
	}

    // Read data from the file
	cv::Mat bigmat, imagePtMat;

	fs["grid_points"] >> gridPoints;
	fs["extrinsic_parameters"] >> bigmat; 
    fs["image_points"] >> imagePtMat;  

	// Convert imagePtMat to std::vector<std::vector<cv::Point2f>>
	imagePoints.resize(imagePtMat.rows);
	for (int i = 0; i < imagePtMat.rows; i++) {
		imagePoints[i] = imagePtMat.row(i);
	}

	// Convert bigmat to rvecs and tvecs
	rvecs.resize(bigmat.rows);
	tvecs.resize(bigmat.rows);
	for (int i = 0; i < bigmat.rows; i++) {
		rvecs[i] = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
		tvecs[i] = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));
	}
	
    fs["image_width"] >> imageSize.width;
    fs["image_height"] >> imageSize.height;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
	// Close the file storage
	fs.release();

	// You can print or process the variables as needed
	//std::cout << "Image Size: " << imageSize << std::endl;
	//std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
	//std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;
}

std::vector<cv::Point2f> monoCamera::getImagePoint()
{
	using namespace std;
	using namespace cv;
	vector<Point2f> result;
	
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


cv::Mat monoCamera::getProjectMatrix()
{
	cv::Mat P;
	cv::Mat RT;
	cv::hconcat(R, T, RT);
	P = cameraMatrix * RT;

	return P;
}
