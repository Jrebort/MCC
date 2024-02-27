#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "Core.h"
#include "Loader.h"
#include "opencv2/core/core.hpp"

namespace MCC {
	void Loader::traverseFloder()
	{
		using namespace boost::filesystem;
		path p(folderPath);
		for (directory_iterator iter{ p }; iter != directory_iterator{}; ++iter)
		{
			if (is_regular_file(iter->status()) && iter->path().extension() == ".png")
			{
				std::string imgPath = iter->path().string();
				//std::cout << imgPath << std::endl;
				imgFilePath.push_back(imgPath);
			}
		}
	}

	void Loader::writeXml()
	{
		if (imgFilePath.empty())
		{
			std::cerr << "Image paths vector is empty!" << std::endl;
			return;
		}

		std::ofstream out(dataDesPath);
		if (!out.is_open())
		{
			std::cerr << "Failed to open " << dataDesPath << " for writing." << std::endl;
			return;
		}

		out << "<?xml version=\"1.0\"?>" << std::endl;
		out << "<opencv_storage>" << std::endl;
		out << "<images>" << std::endl;
		for (const auto& path : imgFilePath)
		{
			out << "\"" << path << "\"" << std::endl;
		}
		out << "</images>" << std::endl;
		out << "</opencv_storage>" << std::endl;

		out.close();
	}

	void Loader::generateSettingXml()
	{
		cv::FileStorage fs(settingPath, cv::FileStorage::WRITE);

		fs << "Settings" << "{";

		fs << "BoardSize_Width" << 11;
		fs << "BoardSize_Height" << 8;
		fs << "Square_Size" << 1.5;
		fs << "Marker_Size" << 25;
		fs << "Calibrate_Pattern" << "CHESSBOARD";
		fs << "ArUco_Dict_Name" << "DICT_4X4_50";
		fs << "ArUco_Dict_File_Name" << "";
		fs << "Input" << dataDesPath;
		fs << "Input_FlipAroundHorizontalAxis" << 0;
		fs << "Input_Delay" << 100;
		fs << "Calibrate_NrOfFrameToUse" << 100;
		fs << "Calibrate_FixAspectRatio" << 1;
		fs << "Calibrate_AssumeZeroTangentialDistortion" << 1;
		fs << "Calibrate_FixPrincipalPointAtTheCenter" << 1;
		fs << "Write_outputFileName" << cameraParamPath;
		fs << "Write_DetectedFeaturePoints" << 1;
		fs << "Write_extrinsicParameters" << 1;
		fs << "Write_gridPoints" << 1;
		fs << "Show_UndistortedImage" << 1;
		fs << "Calibrate_UseFisheyeModel" << 0;
		fs << "Fix_K1" << 1;
		fs << "Fix_K2" << 1;
		fs << "Fix_K3" << 1;
		fs << "Fix_K4" << 1;
		fs << "Fix_K5" << 1;

		fs << "}";

		fs.release();
	}

	bool Loader::isCalibrated()
	{
		using namespace boost::filesystem;
		std::vector < std::string> filenames;
		std::string a("cameraCamera.xml");
		std::string b("dataDes.xml");
		std::string c("setting.xml");
		filenames.push_back(a);
		filenames.push_back(b);
		filenames.push_back(c);

		int countFound = 0;

		path p(folderPath);
		for (directory_iterator it{ p }; it != directory_iterator(); ++it) {
			if (is_regular_file(it->status()) &&
				std::find(filenames.begin(), filenames.end(), it->path().filename().string()) != filenames.end()) {
				countFound++;
			}
		}

		return countFound == filenames.size();
	}

	Loader::~Loader()
	{

	}

	bool Loader::iterateDataFolder(const std::string& foldername, std::vector<std::string>& viewFolders)
	{
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
					//std::cout << it->path() << std::endl;
				}
			}
		}
		return 0;
	}
}
