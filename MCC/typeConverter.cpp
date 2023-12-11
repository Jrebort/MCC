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

void mat3dTovector3d(Mat& mat, vector<Point3d>& stdvector)
{
	for (int i = 0; i < mat.cols; i++)
	{
		stdvector.push_back(Point3d(mat.at<double>(0, i), 
									mat.at<double>(1, i), 
									mat.at<double>(2, i)));
	}
}

void vv3fToV3f(const vector<vector<Point2d>>& input, vector<Point2d>& output)
{
	for (auto& vec : input)
	{
		for (auto& point : vec)
			output.push_back(point);
	}
}

void vv2fToV2f(const vector<vector<Point2d>>& input, vector<Point2d>& output)
{
	for (auto& vec : input)
	{
		for (auto& point : vec)
			output.push_back(point);
	}
}

void MatTov3d(const Mat& input, std::vector<cv::Point3d>& point3d)
{
	for (int i = 0; i < input.cols; i++)
	{
		double x = input.at<double>(0, i);
		double y = input.at<double>(1, i);
		double z = input.at<double>(2, i);
		cv::Point3d pts(x,y,z);
		point3d.push_back(pts);
	}
}

void vv2fToV2d(const std::vector<std::vector<cv::Point2d>>& input, std::vector<cv::Point2d>& output) {
	// 清空输出向量
	output.clear();

	// 遍历所有子向量
	for (const auto& subVec : input) {
		// 遍历子向量中的所有点
		for (const auto& point : subVec) {
			// 将每个 Point2d 转换为 Point2d 并添加到输出向量
			output.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
		}
	}
}

Mat v2dToMat(const vector<Point2d>& input)
{
	unsigned int N = input.size();
	Mat result(2, N, CV_64F);

	unsigned int col = 0;
	for (auto& point : input)
	{
		result.at<double>(0, col) = point.x;
		result.at<double>(1, col) = point.y;
		col++;
	}

	return result;
}

Mat v2fToMat(const vector<Point2d>& input)
{
	unsigned int N = input.size();
	Mat result(2, N, CV_64F);

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
