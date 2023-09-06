#pragma once
#include <ceres/ceres.h>
#include "problem.h"

class BASolver
{
public:
	ceres::Problem problem;

public:
	BASolver(Problem& problem);
	void Solve() {}
};
