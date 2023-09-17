#pragma once 
#include <vector>
#include <string>

class Dataset
{
private:
	std::string folderPath;
	std::vector<std::string> imgFilePath;
	std::string dataDesPath;
	std::string settingPath;
	std::string cameraParamPath;

public:
	Dataset(const std::string& dataFolderPath) 
		:folderPath(dataFolderPath) 
	{
		settingPath = folderPath + "/setting.xml";
		dataDesPath = folderPath + "/dataDes.xml";
		cameraParamPath = folderPath + "/cameraCamera.xml";
	};
	~Dataset();
	unsigned int getImgNum() { return imgFilePath.size(); };
	std::string getSettingPath() { return settingPath; };
	std::string getDataDesPath() { return dataDesPath; };
	std::string getCameraParamPath() { return cameraParamPath; };
	bool isCalibrated();
	void traverseFloder();
	void writeXml();
	void generateSettingXml();
};