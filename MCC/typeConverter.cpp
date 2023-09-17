#include "typeConverter.h"

using namespace std;
using namespace cv;

void mat3dTovector3f(Mat& mat, vector<Point3f>& stdvector)
{
	for (int i = 0; i < mat.cols; i++)
	{
		stdvector.push_back(Point3f(mat.at<double>(0, i), 
									mat.at<double>(1, i), 
									mat.at<double>(2, i)));
	}
}


void vv3fToV3f(const vector<vector<Point2f>>& input, vector<Point2f>& output)
{
	for (auto& vec : input)
	{
		for (auto& point : vec)
			output.push_back(point);
	}
}

void vv2fToV2f(const vector<vector<Point2f>>& input, vector<Point2f>& output)
{
	for (auto& vec : input)
	{
		for (auto& point : vec)
			output.push_back(point);
	}
}

Mat v2fToMat(const vector<Point2f>& input)
{
	unsigned int N = input.size();
	Mat result(3, N, CV_64F);

	unsigned int col = 0;
	for (auto& point : input)
	{
		result.at<double>(0, col) = point.x;
		result.at<double>(1, col) = point.y;
		col++;
	}

	return result;
}

Mat v3fToMat(const vector<Point3f>& input)
{
	unsigned int N = input.size();
	Mat result(3, N, CV_64F);

	unsigned int col = 0;
	for (auto& point : input)
	{
		result.at<double>(0, col) = point.x;
		result.at<double>(1, col) = point.y;
		result.at<double>(2, col) = point.z;
		col++;
	}

	return result;
}
