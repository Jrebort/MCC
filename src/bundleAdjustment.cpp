#include "bundleAdjustment.h"
#include "reprojectionCost.h"

void BASolver::Solve(Problem& multiCCproblem)
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
		//std::cout << i << std::endl;
		ceres::CostFunction* cost_function;

		// Each Residual block takes a point and a camera as input
		// and outputs a 2 dimensional Residual
		cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);

		// If enabled use Huber's loss function.
		ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

		// Each observation corresponds to a pair of a camera and a point
		// which are identified by camera_index()[i] and point_index()[i]
		// respectively.
		double* camera = cameras + camera_block_size * multiCCproblem.camera_index()[i];

		double* point = points + point_block_size * multiCCproblem.point_index()[i];

		problem.AddResidualBlock(cost_function, loss_function, camera, point);
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
	options.max_num_iterations = 1000;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
}

