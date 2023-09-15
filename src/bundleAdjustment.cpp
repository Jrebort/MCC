#include "bundleAdjustment.h"
#include "reprojectionCost.h"

namespace MCC {

	void BASolver::Solve(Problem& multiCCproblem, bool isFixedPoint3d, bool isFixCamera)
	{
		const int point_block_size = multiCCproblem.point_block_size();
		const int camera_block_size = multiCCproblem.camera_block_size();
		double* points = multiCCproblem.mutable_points();
		double* cameras = multiCCproblem.mutable_cameras();

		// Observations is 2 * num_observations long array observations
		// [u_1, u_2, ... u_n], where each u_i is two dimensional, the x
		// and y position of the observation.
		const double* observations = multiCCproblem.observations();
		ceres::Problem problem;

		for (int i = 0; i < multiCCproblem.num_observations(); ++i) {
			ceres::CostFunction* cost_function;

			// If enabled use Huber's loss function.
			ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

			// Each observation corresponds to a pair of a camera and a point
			// which are identified by camera_index()[i] and point_index()[i]
			// respectively.
			double* camera = cameras + camera_block_size * multiCCproblem.camera_index()[i];

			double* point = points + point_block_size * multiCCproblem.point_index()[i];

			//if (multiCCproblem.camera_index()[i] != 5)
			//	continue;
			//using namespace cv;
			double* fx = camera;
			double* fy = camera + 1;
			double* cx = camera + 2;
			double* cy = camera + 3;
			double* k1 = camera + 4;
			double* k2 = camera + 5;
			double* p1 = camera + 6;
			double* p2 = camera + 7;
			double* k3 = camera + 8;
			double* r = camera + 9;
			double* t = camera + 12;

			// __debug__Begin
			//cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << *f, 0, *cx, 0, *f, *cy, 0, 0, 1);
			//cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << *k1, *k2, 0, 0, *k3);
			//cv::Mat objectPoints = (cv::Mat_<double>(3, 1) << point[0], point[1], point[2]);
			//cv::Mat rvec = (cv::Mat_<double>(3, 1) << r[0], r[1], r[2]);
			//cv::Mat tvec = (cv::Mat_<double>(3, 1) << t[0], t[1], t[2]);
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
			//std::cout << observations[2 * i] <<" "<< observations[2 * i + 1] << std::endl;
			// __debug__End

			// Each Residual block takes a point and a camera as input
			// and outputs a 2 dimensional Residual
			if (isFixedPoint3d)
			{
				cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
				problem.AddResidualBlock(cost_function, loss_function, fx, fy, cx, cy, k1, k2, p1, p2, k3, r, t, point);
				problem.SetParameterBlockConstant(point);
				problem.SetParameterBlockConstant(fx);
				problem.SetParameterBlockConstant(fy);
				problem.SetParameterBlockConstant(cx);
				problem.SetParameterBlockConstant(cy);
				//problem.SetParameterBlockConstant(p1);
				//problem.SetParameterBlockConstant(p2);
			}
			else
			{
				cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
				problem.AddResidualBlock(cost_function, loss_function, camera, point);
			}
		}

		// show some information here ...
		std::cout << "problem file loaded..." << std::endl;
		std::cout << "problem have " << multiCCproblem.num_cameras() << " cameras and "
			<< multiCCproblem.num_points() << " points. " << std::endl;
		std::cout << "Forming " << multiCCproblem.num_observations() << " observations. " << std::endl;

		std::cout << "Solving ceres BA ... " << std::endl;
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		options.max_num_iterations = 150;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		std::cout << summary.FullReport() << "\n";
	}
}
