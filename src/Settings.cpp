#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/objdetect/charuco_detector.hpp"
#include <iostream>

#include "Settings.h"

Settings::Settings() : goodInput(false) {}

void Settings::write(cv::FileStorage& fs) const
{
	fs << "{"
		<< "BoardSize_Width" << boardSize.width
		<< "BoardSize_Height" << boardSize.height
		<< "Square_Size" << squareSize
		<< "Marker_Size" << markerSize
		<< "Calibrate_Pattern" << patternToUse
		<< "ArUco_Dict_Name" << arucoDictName
		<< "ArUco_Dict_File_Name" << arucoDictFileName
		<< "Calibrate_NrOfFrameToUse" << nrFrames
		<< "Calibrate_FixAspectRatio" << aspectRatio
		<< "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
		<< "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

		<< "Write_DetectedFeaturePoints" << writePoints
		<< "Write_extrinsicParameters" << writeExtrinsics
		<< "Write_gridPoints" << writeGrid
		<< "Write_outputFileName" << outputFileName

		<< "Show_UndistortedImage" << showUndistorted

		<< "Input_FlipAroundHorizontalAxis" << flipVertical
		<< "Input" << input
		<< "}";
}

void Settings::read(const cv::FileNode& node)
{
	node["BoardSize_Width"] >> boardSize.width;
	node["BoardSize_Height"] >> boardSize.height;
	node["Calibrate_Pattern"] >> patternToUse;
	node["ArUco_Dict_Name"] >> arucoDictName;
	node["ArUco_Dict_File_Name"] >> arucoDictFileName;
	node["Square_Size"] >> squareSize;
	node["Marker_Size"] >> markerSize;
	node["Calibrate_NrOfFrameToUse"] >> nrFrames;
	node["Calibrate_FixAspectRatio"] >> aspectRatio;
	node["Write_DetectedFeaturePoints"] >> writePoints;
	node["Write_extrinsicParameters"] >> writeExtrinsics;
	node["Write_gridPoints"] >> writeGrid;
	node["Write_outputFileName"] >> outputFileName;
	node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
	node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
	node["Calibrate_UseFisheyeModel"] >> useFisheye;
	node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
	node["Show_UndistortedImage"] >> showUndistorted;
	node["Input"] >> input;
	node["Input_Delay"] >> delay;
	node["Fix_K1"] >> fixK1;
	node["Fix_K2"] >> fixK2;
	node["Fix_K3"] >> fixK3;
	node["Fix_K4"] >> fixK4;
	node["Fix_K5"] >> fixK5;

	validate();
}

void Settings::validate()
{
	using namespace cv;
	using namespace std;
	goodInput = true;
	if (boardSize.width <= 0 || boardSize.height <= 0)
	{
		cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
		goodInput = false;
	}
	if (squareSize <= 10e-6)
	{
		cerr << "Invalid square size " << squareSize << endl;
		goodInput = false;
	}
	if (nrFrames <= 0)
	{
		cerr << "Invalid number of frames " << nrFrames << endl;
		goodInput = false;
	}


    if (isListOfImages(input) && readStringList(input, imageList))
    {
        nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
    }

	flag = 0;
	if (calibFixPrincipalPoint) flag |= cv::CALIB_FIX_PRINCIPAL_POINT;
	if (calibZeroTangentDist)   flag |= cv::CALIB_ZERO_TANGENT_DIST;
	if (aspectRatio)            flag |= cv::CALIB_FIX_ASPECT_RATIO;
	if (fixK1)                  flag |= cv::CALIB_FIX_K1;
	if (fixK2)                  flag |= cv::CALIB_FIX_K2;
	if (fixK3)                  flag |= cv::CALIB_FIX_K3;
	if (fixK4)                  flag |= cv::CALIB_FIX_K4;
	if (fixK5)                  flag |= cv::CALIB_FIX_K5;

	if (useFisheye) {
		// the fisheye model has its own enum, so overwrite the flags
		flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
		if (fixK1)                   flag |= cv::fisheye::CALIB_FIX_K1;
		if (fixK2)                   flag |= cv::fisheye::CALIB_FIX_K2;
		if (fixK3)                   flag |= cv::fisheye::CALIB_FIX_K3;
		if (fixK4)                   flag |= cv::fisheye::CALIB_FIX_K4;
		if (calibFixPrincipalPoint) flag |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
	}

	calibrationPattern = NOT_EXISTING;
	if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
	if (!patternToUse.compare("CHARUCOBOARD")) calibrationPattern = CHARUCOBOARD;
	if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
	if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
	if (calibrationPattern == NOT_EXISTING)
	{
		cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
		goodInput = false;
	}
	atImageList = 0;
}

cv::Mat Settings::nextImage()
{
	using namespace cv;
	Mat result;
	if (atImageList < imageList.size())
		result = imread(imageList[atImageList++], IMREAD_COLOR);

	return result;
}

bool Settings::ptrToBeginImage()
{
	atImageList = 0;
	return 0;
}

bool Settings::readStringList(const std::string& filename, std::vector<std::string>& l)
{
	using namespace cv;
	using namespace std;
	l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

bool Settings::isListOfImages(const std::string& filename)
{
	using namespace std;
	string s(filename);
	// Look for file extension
	if (s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos)
		return false;
	else
		return true;
}
