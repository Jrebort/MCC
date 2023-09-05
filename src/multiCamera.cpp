#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/sfm/projection.hpp>
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

void multiCamera::writeCameraParamter()
{
}

void multiCamera::pnpOptimization()
{	
	using namespace cv;
	using namespace sfm;
	const unsigned int patternNum = cameraMatrix[0].imagePoints.size();
	const monoCamera& cameraBase = cameraMatrix[0];
	for (int i = 0; i < patternNum; i++)
	{
		// set local variable
		Mat r;
		Mat t = cameraBase.tvecs[i].t();
		Mat point3d(cameraBase.gridPoints);
		point3d = point3d.reshape(3, 88);

		Rodrigues(cameraBase.rvecs[i], r, noArray());

		Mat T(3, 4, r.type());
		r.copyTo( T(Rect(0, 0, 3, 3)));
		t.copyTo( T(Rect(3, 0, 1, 3)));

		worldPoint = r.mul(point3d) + repeat(t, 1, 88);
		transform(point3d, worldPoint, T);
	}
}

void multiCamera::visCameraPose()
{
	//using namespace cv;
	//using namespace std;
	//	
	//viz::Viz3d window("Coordinate Frame");
	//window.setWindowSize(Size(500, 500));
	//window.setWindowPosition(Point(150, 150));
	//window.setBackgroundColor(); // black by default

	//cout << "Recovering cameras ... ";
	//vector<Affine3d> path;
	//for (size_t i = 0; i < Rs_est.size(); ++i)
	//path.push_back(Affine3d(Rs_est[i],ts_est[i]));
	//cout << "[DONE]" << endl;

	//if (path.size() > 0)
	//{
	//	cout << "Rendering Cameras ... ";
	//	window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
	//	window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
	//	window.setViewerPose(path[0]);
	//	cout << "[DONE]" << endl;
	//}
	//else
	//{
	//	cout << "Cannot render the cameras: Empty path" << endl;
	//}
	//cout << endl << "Press 'q' to close each windows ... " << endl;
	//window.spin();
}

