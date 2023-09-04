#include "multiCamera.h"

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
