#include <iostream>
#include <opencv2/calib3d.hpp>
#include "ceres/ceres.h"
#include "rotation.h"

class SnavelyReprojectionError {
public:
	SnavelyReprojectionError(double observation_x, double observation_y) : observed_x(observation_x),
		observed_y(observation_y) {}

	template<typename T>
	bool operator()(const T* const f,
		const T* const cx,
		const T* const cy,
		const T* const k1,
		const T* const k2,
		const T* const p1,
		const T* const p2,
		const T* const k3,
		const T* const r,
		const T* const t,
		const T* const point,
		T* residuals) const {
		// camera[0,1,2] are the angle-axis rotation
		T predictions[2];
		//std::cout << point[0] << std::endl;
		//std::cout << point[1] << std::endl;
		//std::cout << point[2] << std::endl;
		CamProjectionWithDistortion(f[0], cx[0], cy[0], k1[0], k2[0], p1[0], p2[0], k3[0], r, t, point, predictions);
		//using namespace cv;
		//std::cout << f[0] << std::endl;
		//std::cout << cx[0] << std::endl;
		//std::cout << cy[0] << std::endl;
		//std::cout << k1[0] << std::endl;
		//std::cout << k2[0] << std::endl;
		//std::cout << p1[0] << std::endl;
		//std::cout << p2[0] << std::endl;
		//std::cout << k3[0] << std::endl;
		//std::cout << r[0] << std::endl;
		//std::cout << r[1] << std::endl;
		//std::cout << r[2] << std::endl;
		//std::cout << t[0] << std::endl;
		//std::cout << t[1] << std::endl;
		//std::cout << t[2] << std::endl;
		//std::cout << point[0] << std::endl;
		//std::cout << point[1] << std::endl;
		//std::cout << point[2] << std::endl;
		//cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 30332.1, 0, 1023.5, 0, 30332.1, 682.5, 0, 0, 1);
		//cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.267748, 2.75283, 0, 0, 2.6052);
		//cv::Mat objectPoints = (cv::Mat_<double>(3, 1) << 51.3254, 42.2047, 421.27);
		//cv::Mat rvec = (cv::Mat_<double>(3, 1) << -0.32888, -0.770669, -2.51629);
		//cv::Mat tvec = (cv::Mat_<double>(3, 1) << 44.0088, -149.742, 164.385);
		//cv::Mat imagePoints;

		//cv::projectPoints(objectPoints,
		//	rvec,
		//	tvec,
		//	cameraMatrix,
		//	distCoeffs,
		//	imagePoints,
		//	noArray(),
		//	0.0
		//);
		//std::cout << imagePoints << std::endl;

		residuals[0] = predictions[0] - T(observed_x);
		residuals[1] = predictions[1] - T(observed_y);

		return true;
	}

	// camera : 14 dims array
	// [0-3] : f, cx, cy
	// [3-8] : k1, k2, p1, p2, k3
	// [8-11] : angle-axis rotation
	// [11-14] : translateion
	// point : 3D location.
	// predictions : 2D predictions with center of the image plane.
	template<typename T>
	static inline bool CamProjectionWithDistortion(const T& const focal,
		const T& const cx,
		const T& const cy,
		const T& const k1,
		const T& const k2,
		const T& const p1,
		const T& const p2,
		const T& const k3,
		const T* const r,
		const T* const t,
		const T* const point, T* predictions) {
		// Rodrigues' formula
		T p[3];

		//std::cout << point[0] << std::endl;
		//std::cout << point[1] << std::endl;
		//std::cout << point[2] << std::endl;
		AngleAxisRotatePoint(r, point, p);
		// camera[3,4,5] are the translation
		p[0] += t[0];
		p[1] += t[1];
		p[2] += t[2];

		// Compute the center fo distortion
		T xp = p[0] / p[2];
		T yp = p[1] / p[2];

		// Apply second and fourth order radial distortion
		T r2 = xp * xp + yp * yp;
		T RadialCoff = T(1.0) + r2 * (k1 + k2 * r2 + k3 * r2 * r2);
		T Xtangential = T(2.0) * p1 * xp * yp + p2 * (r2 + T(2.0) * xp * xp);
		T Ytangential = T(2.0) * p2 * xp * yp + p1 * (r2 + T(2.0) * yp * yp);

		T xd = RadialCoff * xp + Xtangential;
		T yd = RadialCoff * yp + Ytangential;

		predictions[0] = focal * xd + cx;
		predictions[1] = focal * yd + cy;

		return true;
	}

	static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3>(
			new SnavelyReprojectionError(observed_x, observed_y)));
	}

private:
	double observed_x;
	double observed_y;
	bool fixCameraParameter[14] = { 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
};