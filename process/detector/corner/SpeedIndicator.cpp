#include "SpeedIndicator.h"

static const std::vector<std::vector<cv::Point2f>> gBfs =
{
	//{
	//	// 全体
	//	cv::Point2f(1278, 329),
	//	cv::Point2f(1489, 718),
	//	cv::Point2f(556, 337),
	//	cv::Point2f(431, 719),
	//},
	{
		// 左端
		cv::Point2f(656, 353),
		cv::Point2f(669, 720),
		cv::Point2f(549, 345),
		cv::Point2f(483, 714),
	},
	{
		// 中央
		cv::Point2f(1078, 359),
		cv::Point2f(946, 717),
		cv::Point2f(1160, 355),
		cv::Point2f(1108, 724)
	},
	{
		// 右端
		cv::Point2f(1212, 355),
		cv::Point2f(1340, 718),
		cv::Point2f(1290, 349),
		cv::Point2f(1489, 718)
	},
};

static const std::vector<std::vector<cv::Point2f>> gAfs =
{
	//{
	//	// 全体
	//	cv::Point2f(2585, 310),
	//	cv::Point2f(941, 2944),
	//	cv::Point2f(1638, 9),
	//	cv::Point2f(270, 2674),
	//},
	{
		//左端
		cv::Point2f(2162, 134),
		cv::Point2f(516, 3288),
		cv::Point2f(1980, 29),
		cv::Point2f(335, 3093),
	},
	{
		// 中央
		cv::Point2f(2255, 345),
		cv::Point2f(625, 2721),
		cv::Point2f(2342, 383),
		cv::Point2f(689, 2785)
	},
	{
		// 右端
		cv::Point2f(2481, 307),
		cv::Point2f(961, 2748),
		cv::Point2f(2538, 334),
		cv::Point2f(1014, 2793)
	},
};

static const std::vector<cv::Rect2f> gBfRects =
{
	cv::boundingRect(gBfs[0]),
	cv::boundingRect(gBfs[1]),
	cv::boundingRect(gBfs[2]),
};

static double calcDistance(const cv::Point2f& before, const cv::Point2f& after, const double& magni)
{
	auto distVec = after - before;
	auto pixDist = cv::norm(distVec);
	return pixDist * magni;
}

static cv::Point2f getPTransPoint(const cv::Mat& trans, const cv::Point2f& point)
{
	std::vector<cv::Point2f> srcCorners{};
	std::vector<cv::Point2f> dstCorners{};

	srcCorners.push_back(point);
	cv::perspectiveTransform(srcCorners, dstCorners, trans);
	return dstCorners[0];
}

double SpeedIndicator::calcSpeed(const double& sumDelta, const uint64_t& time, const double& fps)
{
	auto speedPerS = (sumDelta / time) * fps;
	return speedPerS * 3600;
}

double SpeedIndicator::calcDelta(const cv::Point2f& oldPoint, const cv::Point2f& newPoint, const double& magni)
{
	size_t roadId = 0;
	for (size_t i = 0; i < gBfRects.size(); i++)
	{
		if (gBfRects[i].contains(oldPoint))
		{
			roadId = i;
			break;
		}
	}

	auto trans = cv::getPerspectiveTransform(gBfs[roadId], gAfs[roadId]);
	const auto& before = getPTransPoint(trans, oldPoint);
	const auto& after = getPTransPoint(trans, newPoint);

	return ::calcDistance(before, after, magni) * 0.001;
}