#pragma once 
#include <vector>
#include <string>

namespace MCC {
	class Loader
	{
	private:
		std::string folderPath;
		std::vector<std::string> imgFilePath;
		std::string dataDesPath;
		std::string settingPath;
		std::string cameraParamPath;

	public:
		Loader(const std::string& dataFolderPath)
			:folderPath(dataFolderPath)
		{
			settingPath = folderPath + "/setting.xml";
			dataDesPath = folderPath + "/dataDes.xml";
			cameraParamPath = folderPath + "/cameraCamera.xml";
		};
		~Loader();

		unsigned int getImgNum() { return imgFilePath.size(); };
		std::string getSettingPath() { return settingPath; };
		std::string getDataDesPath() { return dataDesPath; };
		std::string getCameraParamPath() { return cameraParamPath; };
		bool isCalibrated();
		void traverseFloder();
		void writeXml();
		void generateSettingXml();
		static bool iterateDataFolder(const std::string& DataPath, std::vector<std::string>& viewFolders);
	};
}