#include <iostream>
#include <opencv2/calib3d.hpp>
#include "ceres/ceres.h"
#include "rotation.h"

class SnavelyReprojectionError {
public:
	SnavelyReprojectionError(double observation_x, double observation_y) : observed_x(observation_x),
		observed_y(observation_y) {}

	template<typename T>
	bool operator()(const T* const fx,
		const T* const fy,
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
		CamProjectionWithDistortion(fx[0], fy[0], cx[0], cy[0], k1[0], k2[0], p1[0], p2[0], k3[0], r, t, point, predictions);

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
	static inline bool CamProjectionWithDistortion(const T& const fx,
		const T& const fy,
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
		
		// __debug__Begin
		//std::cout << point[0] << std::endl;
		//std::cout << point[1] << std::endl;
		//std::cout << point[2] << std::endl;
		// __debug__End

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

		predictions[0] = fx * xd + cx;
		predictions[1] = fy * yd + cy;

		return true;
	}

	static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3>(
			new SnavelyReprojectionError(observed_x, observed_y)));
	}

private:
	double observed_x;
	double observed_y;
};