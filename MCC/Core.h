#pragma once
#include <iostream>

#define ASSERT(x, msg) if (x) { std::cout << msg << std::endl;\
								__debugbreak();}

#define Step(msg) std::cout << std::endl;\
			std::cout << "##### " << "Step " << STEPNUM << ": " <<  msg << " #####" <<std::endl;\
			STEPNUM += 1;
				