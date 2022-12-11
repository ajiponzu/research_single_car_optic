#include "CenterDetector.h"
#include "../../utility/GuiHandler.h"
#include "center/SpeedIndicator.h"

cv::Mat gKernel = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(3, 3));
cv::Mat gRoadMask = cv::imread("resources/back_kaikai.png", cv::IMREAD_COLOR);

cv::Point minPoint(const std::vector<cv::Point>& contours)
{
	auto minx = contours.at(0).x;
	auto miny = contours.at(0).y;

	for (const auto& point : contours)
	{
		if (minx > point.x)
			minx = point.x;

		if (miny > point.y)
			miny = point.y;
	}

	return cv::Point(minx, miny);
}

cv::Point maxPoint(const std::vector<cv::Point>& contours)
{
	auto maxx = contours.at(0).x;
	auto maxy = contours.at(0).y;

	for (const auto& point : contours)
	{
		if (maxx < point.x)
			maxx = point.x;

		if (maxy < point.y)
			maxy = point.y;
	}

	return cv::Point(maxx, maxy);
}

cv::Point calcRectCenter(const cv::Rect& rect)
{
	auto cx = rect.x + rect.width / 2;
	auto cy = rect.y + rect.height / 2;

	return cv::Point(cx, cy);
}

void localBinarize(cv::Mat& local_ret)
{
	cv::threshold(local_ret, local_ret, 0, 255, cv::THRESH_OTSU);
	cv::morphologyEx(local_ret, local_ret, cv::MORPH_CLOSE, gKernel, cv::Point(-1, -1), 2);
	cv::cvtColor(local_ret, local_ret, cv::COLOR_GRAY2BGR);
}

std::pair<cv::Rect, cv::Point2d> moveCenterPoint(const cv::Mat& img, const cv::Mat& local_ret, cv::Rect& rect)
{
	cv::Rect target_rect;
	cv::Point2d ret_center;
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(local_ret, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cv::Point correct_center_delta;
	//double min_center_dist = cv::norm(cv::Point(rect.width, rect.height));
	double min_center_dist = cv::norm(cv::Point(img.cols, img.rows));
	auto cur_center = calcRectCenter(rect) - rect.tl();

	for (const auto& contour : contours)
	{
		cv::Rect contour_rect(minPoint(contour), maxPoint(contour));
		auto center_delta = calcRectCenter(contour_rect) - cur_center;
		auto center_dist = cv::norm(center_delta);

		const auto lamda = [&]
		{
			correct_center_delta = center_delta;
			target_rect = contour_rect;
			auto moments = cv::moments(contour);
			ret_center = cv::Point2d(moments.m10 / moments.m00, moments.m01 / moments.m00);
			ret_center += static_cast<cv::Point2d>(rect.tl());
		};

		if (contour_rect.contains(cur_center))
		{
			lamda();
			break;
		}

		if (center_dist < min_center_dist)
		{
			min_center_dist = center_dist;
			lamda();
		}
	}
	rect.x = std::clamp(rect.x + correct_center_delta.x, 0, img.cols - rect.width);
	rect.y = std::clamp(rect.y + correct_center_delta.y, 0, img.rows - rect.height);

	return { target_rect, ret_center };
}

std::vector<std::vector<Detection>> CenterDetector::Run(const cv::Mat& img, cv::Rect& rect, const bool& reset)
{
	std::vector<std::vector<Detection>> results{};
	const auto& frame_count = GuiHandler::GetFrameCount();

	mptr_bgController->Create(img, frame_count);
	if (frame_count < static_cast<uint64_t>(mptr_bgController->GetExHistory()))
		return results;

	if (rect.width == 0 || rect.height == 0)
		return results;

	if (reset)
	{
		m_startFrameCount = GuiHandler::GetFrameCount();
		m_trackingCount = 0;
		m_sumDelta = 0.0;
		m_speed = 0.0;
	}

	auto [subtract, target_rect] = BgSubtract(img, mptr_bgController->GetBg(), rect);
	if (target_rect.width == 0 || target_rect.height == 0)
	{
		std::cout << "target_rect is not found." << std::endl;
		rect = cv::Rect();
		m_trackingCount = 0;
		m_sumDelta = 0.0;
		m_speed = 0.0;
		return results;
	}

	if (!reset)
		CalcSpeed(target_rect);
	else
		m_prevCenter = m_center;

	m_prevSubtracted = m_Subtracted.clone();
	results.push_back({ target_rect });

	return results;
}

std::pair<cv::Mat, cv::Rect> CenterDetector::BgSubtract(const cv::Mat& img, const cv::Mat& bg, cv::Rect& rect)
{
	cv::Mat ret{}, mask{};
	cv::Rect target_rect;

	if (!GuiHandler::GetDetectionRect().contains(calcRectCenter(rect)))
		return { ret, target_rect };

	cv::absdiff(img, bg, ret);
	cv::bitwise_and(ret, gRoadMask, ret);
	cv::cvtColor(ret, ret, cv::COLOR_BGR2GRAY);

	auto local_ret = ret(rect).clone();
	localBinarize(local_ret);
	cv::bitwise_and(local_ret, gRoadMask(rect), local_ret);

	cv::cvtColor(local_ret, local_ret, cv::COLOR_BGR2GRAY);
	auto center_pair = moveCenterPoint(img, local_ret, rect);
	target_rect = center_pair.first;
	m_center = center_pair.second;

	if (target_rect.width == 0 || target_rect.height == 0)
		return { local_ret, target_rect };

	local_ret = ret(rect).clone();
	localBinarize(local_ret);
	cv::bitwise_and(local_ret, gRoadMask(rect), local_ret);

	auto org_target_rect = target_rect + rect.tl();
	m_Subtracted = cv::Mat::zeros(img.size(), CV_8UC3);
	local_ret(target_rect).copyTo(m_Subtracted(org_target_rect));

	return { local_ret, target_rect };
}

void CenterDetector::CalcSpeed(const cv::Rect& target_rect)
{
	m_trackingCount++;
	m_sumDelta = SpeedIndicator::calcDelta(m_prevCenter, m_center, 0.1);
	auto speed = SpeedIndicator::calcSpeed(m_sumDelta, m_trackingCount, GuiHandler::GetFPS());
	//if (GuiHandler::GetKeyEvent(static_cast<int>(' ')))
	if (true)
	{
		std::cout << "current_frame_count: " << GuiHandler::GetFrameCount() << std::endl;
		std::cout << "trace_time: " << m_trackingCount << std::endl;
		std::cout << "answer_cul_speed: " << speed << std::endl;
		std::cout << "sumDelta: " << m_sumDelta << std::endl;
	}
}