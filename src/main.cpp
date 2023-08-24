#include <iostream>
#include <opencv2/imgcodecs.hpp>



int main()
{
	cv::Mat img = cv::imread("./test.png", cv::IMREAD_GRAYSCALE);

	std::cout << "Test Code" << std::endl;
	std::cout << std::cin.get();
}