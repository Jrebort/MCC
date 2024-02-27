#pragma once
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

void mat3dTovector3f(Mat& mat, vector<Point3f>& stdvector);
void mat3dTovector3d(Mat& mat, vector<Point3d>& stdvector);

void vv3fToV3f(const vector<vector<Point3f>>& input, vector<Point3f>& output);
void vv2fToV2f(const vector<vector<Point2d>>& input, vector<Point2d>& output);
void vv2dToV2d(const std::vector<std::vector<cv::Point2d>>& input, std::vector<cv::Point2d>& output);

Mat v2fToMat(const vector<Point2d>& input);
Mat v3fToMat(const vector<Point3f>& input);
Mat v2dToMat(const vector<Point2d>& input);
void MatTov3d(const Mat& input, std::vector<cv::Point3d>& point3d);
