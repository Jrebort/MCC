#include <string>
#include <iostream>
#include <fstream>

#include "monoCameraCalibration.h"

int main()
{
	const std::string filePath = "./setting/testSetting.xml";
	const int winSize = 11;
	monoCameraCalibration calibrater(filePath, winSize);
	calibrater.calibrate();
	calibrater.showCalibrationResults(DETECTION);
}