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
		cv::Point2f(665, 218),
		cv::Point2f(576, 217),
		cv::Point2f(478, 876),
		cv::Point2f(816, 869),
	},
	{
		// 中央
		cv::Point2f(1042, 320),
		cv::Point2f(1147, 317),
		cv::Point2f(831, 864),
		cv::Point2f(1011, 894)
	},
	{
		// 右端
		cv::Point2f(1158, 328),
		cv::Point2f(1266, 325),
		cv::Point2f(1297, 904),
		cv::Point2f(1516, 909)
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
		cv::Point2f(3160, 3207),
		cv::Point2f(2976, 3137),
		cv::Point2f(565, 7610),
		cv::Point2f(831, 7759),
	},
	{
		// 中央
		cv::Point2f(2625, 5098),
		cv::Point2f(2757, 5177),
		cv::Point2f(805, 7785),
		cv::Point2f(928, 7846)
	},
	{
		// 右端
		cv::Point2f(2766, 5194),
		cv::Point2f(2897, 5264),
		cv::Point2f(1042, 7916),
		cv::Point2f(1147, 7995)
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

	//std::cout << "before: " << before << std::endl;
	//std::cout << "after: " << after << std::endl;

	return ::calcDistance(before, after, magni) * 0.001;
}