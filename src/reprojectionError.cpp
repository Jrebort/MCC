#include <opencv2/calib3d.hpp>

#include "reprojectionError.h"

double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints, 
	const std::vector<std::vector<cv::Point2f> >& imagePoints, 
	const std::vector<cv::Mat>& rvecs, 
	const std::vector<cv::Mat>& tvecs, 
	const cv::Mat& cameraMatrix, 
	const cv::Mat& distCoeffs, 
	std::vector<float>& perViewErrors, 
	bool fisheye)
{
	std::vector<cv::Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		if (fisheye)
		{
			cv::fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs);
		}
		else
		{
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}


double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints, 
	const std::vector<std::vector<cv::Point2f> >& imagePoints, 
	const cv::Mat& rvecs, 
	const cv::Mat& tvecs, 
	const cv::Mat& cameraMatrix, 
	const cv::Mat& distCoeffs, 
	std::vector<float>& perViewErrors, 
	bool fisheye)
{
	std::vector<cv::Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		if (fisheye)
		{
			cv::fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs, tvecs, cameraMatrix,
				distCoeffs);
		}
		else
		{
			projectPoints(objectPoints[i], rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

double computeReprojectionErrors(const std::vector<cv::Point3f>& objectPoints,
	const std::vector<cv::Point2f>& imagePoints,
	const cv::Mat& rvecs,
	const cv::Mat& tvecs,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	bool fisheye)
{
	std::vector<cv::Point2f> imagePoints2;
	double err = 0;

	if (fisheye)
	{
		cv::fisheye::projectPoints(objectPoints, imagePoints2, rvecs, tvecs, cameraMatrix,
			distCoeffs);
	}
	else
	{
		projectPoints(objectPoints, rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints2);
	}
	err = norm(imagePoints, imagePoints2, cv::NORM_L2);
	size_t n = objectPoints.size();
	err = (float)std::sqrt(err * err / n);
	return err;
}


