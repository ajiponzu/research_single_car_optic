#pragma once
#include <opencv2/opencv.hpp>

namespace SpeedIndicator
{
	/* 距離加算して後で時速を求める場合 */
	double calcSpeed(const double& sumDelta, const uint64_t& time, const double& fps);
	double calcDelta(const cv::Point2f& oldPoint, const cv::Point2f& newPoint, const double& magni);
	/* end */
};
