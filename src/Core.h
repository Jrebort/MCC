#pragma once
#include <iostream>

#define ASSERT(x, msg) if (x) { std::cout << msg << std::endl;\
								__debugbreak();}