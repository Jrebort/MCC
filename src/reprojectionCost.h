#include <iostream>
#include "ceres/ceres.h"
#include "rotation.h"

class SnavelyReprojectionError {
public:
	SnavelyReprojectionError(double observation_x, double observation_y) : observed_x(observation_x),
		observed_y(observation_y) {}

	SnavelyReprojectionError(double observation_x, double observation_y,const double* point) : observed_x(observation_x),
		observed_y(observation_y), point(point) {}

	template<typename T>
	bool operator()(const T* const camera,
		const T* const point,
		T* residuals) const {
		// camera[0,1,2] are the angle-axis rotation
		T predictions[2];
		CamProjectionWithDistortion(camera, point, predictions);
		residuals[0] = predictions[0] - T(observed_x);
		residuals[1] = predictions[1] - T(observed_y);

		return true;
	}

	template<typename T>
	bool operator()(const T* const camera,
		T* residuals) const {
		// camera[0,1,2] are the angle-axis rotation
		T predictions[2];
		CamProjectionWithDistortion(camera, point, predictions);
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
	static inline bool CamProjectionWithDistortion(const T* camera, const T* point, T* predictions) {
		// Rodrigues' formula
		T p[3];
		AngleAxisRotatePoint(camera, point, p);
		// camera[3,4,5] are the translation
		p[0] += camera[11];
		p[1] += camera[12];
		p[2] += camera[13];

		// Compute the center fo distortion
		T xp = p[0] / p[2];
		T yp = p[1] / p[2];

		// Apply second and fourth order radial distortion
		const T& k1 = camera[3];
		const T& k2 = camera[4];
		const T& k3 = camera[7];
		const T& p1 = camera[5];
		const T& p2 = camera[6];

		T r2 = xp * xp + yp * yp;
		T RadialCoff = T(1.0) + r2 * (k1 + k2 * r2 + k3 * r2 * r2);
		T Xtangential = T(2.0) * p1 * xp * yp + p2 * (r2 + T(2.0) * xp * xp);
		T Ytangential = T(2.0) * p2 * xp * yp + p1 * (r2 + T(2.0) * yp * yp);

		const T& focal = camera[0];
		predictions[0] = focal * RadialCoff * xp + Xtangential;
		predictions[1] = focal * RadialCoff * yp + Ytangential;

		return true;
	}

	static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 14, 3>(
			new SnavelyReprojectionError(observed_x, observed_y)));
	}

	static ceres::CostFunction* Create(const double observed_x, const double observed_y, const double* point) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 14, 3>(
			new SnavelyReprojectionError(observed_x, observed_y, point)));
	}

private:
	double observed_x;
	double observed_y;
	const double* point;
};