#pragma once
#include <ceres/ceres.h>
#include "problem.h"

class BASolver
{
public:
	static void Solve(Problem& problem, bool isFixedPoint3d, bool isFixCamera);
};
