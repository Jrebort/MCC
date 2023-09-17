#pragma once
#include <opencv2/core.hpp> 

#define CamCaliASSERT(x, message) if(x) { __debugbreak(); \
		std::cout << message << std::endl;}

class Settings
{
public:
	Settings();
    enum Pattern { NOT_EXISTING, CHESSBOARD, CHARUCOBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
	
	void write(cv::FileStorage& fs) const;

	void read(const cv::FileNode& node);

	void validate();

	cv::Mat nextImage();

	bool ptrToBeginImage();
	
	static bool readStringList(const std::string& filename, std::vector<std::string>& l);

	static bool isListOfImages(const std::string& filename);

public:
	cv::Size boardSize;              // The size of the board -> Number of items by width and height
	Pattern calibrationPattern;  // One of the Chessboard, ChArUco board, circles, or asymmetric circle pattern
	float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
	float markerSize;            // The size of a marker in your defined unit (point, millimeter,etc).
	std::string arucoDictName;        // The Name of ArUco dictionary which you use in ChArUco pattern
	std::string arucoDictFileName;    // The Name of file which contains ArUco dictionary for ChArUco pattern
	int nrFrames;                // The number of frames to use from the input for calibration
	float aspectRatio;           // The aspect ratio
	int delay;                   // In case of a video input
	bool writePoints;            // Write detected feature points
	bool writeExtrinsics;        // Write extrinsic parameters
	bool writeGrid;              // Write refined 3D target grid points
	bool calibZeroTangentDist;   // Assume zero tangential distortion
	bool calibFixPrincipalPoint; // Fix the principal point at the center
	bool flipVertical;           // Flip the captured images around the horizontal axis
	std::string outputFileName;       // The name of the file where to write
	bool showUndistorted;        // Show undistorted images after calibration
	std::string input;                // The input ->
	bool useFisheye;             // use fisheye camera model for calibration
	bool fixK1;                  // fix K1 distortion coefficient
	bool fixK2;                  // fix K2 distortion coefficient
	bool fixK3;                  // fix K3 distortion coefficient
	bool fixK4;                  // fix K4 distortion coefficient
	bool fixK5;                  // fix K5 distortion coefficient

	int cameraID;
	std::vector<std::string> imageList;
	bool goodInput;
	int flag;

private:
	size_t atImageList;
    std::string patternToUse;
};

