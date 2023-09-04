#pragma once
#include <vector>
#include "monoCamera.h"

class multiCamera
{
private:
	std::vector<monoCamera> cameraMatrix;
public:
	multiCamera();
	~multiCamera();
	void addCamera(monoCamera& camera);
};