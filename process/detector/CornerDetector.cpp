#include "CornerDetector.h"
#include "../../utility/GuiHandler.h"
#include "corner/SpeedIndicator.h"

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

cv::Rect moveCenterPoint(const cv::Mat& img, const cv::Mat& local_ret, cv::Rect& rect)
{
	cv::Rect target_rect;
	std::vector<std::vector<cv::Point>> contours_list;
	cv::findContours(local_ret, contours_list, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cv::Point correct_center_delta;
	//double min_center_dist = cv::norm(cv::Point(rect.width, rect.height));
	double min_center_dist = cv::norm(cv::Point(img.cols, img.rows));
	auto cur_center = calcRectCenter(rect) - rect.tl();

	for (const auto& contours : contours_list)
	{
		cv::Rect contour_rect(minPoint(contours), maxPoint(contours));
		auto center_delta = calcRectCenter(contour_rect) - cur_center;
		auto center_dist = cv::norm(center_delta);

		if (contour_rect.contains(cur_center))
		{
			correct_center_delta = center_delta;
			target_rect = contour_rect;
			break;
		}

		if (center_dist < min_center_dist)
		{
			min_center_dist = center_dist;
			correct_center_delta = center_delta;
			target_rect = contour_rect;
		}
	}
	rect.x = std::clamp(rect.x + correct_center_delta.x, 0, img.cols - rect.width);
	rect.y = std::clamp(rect.y + correct_center_delta.y, 0, img.rows - rect.height);

	return target_rect;
}

std::vector<std::vector<Detection>> CornerDetector::Run(const cv::Mat& img, cv::Rect& rect, const bool& reset)
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
		m_opticCount = 0;
		m_sumDelta = 0.0;
		m_speed = 0.0;
	}

	auto [subtract, target_rect] = BgSubtract(img, mptr_bgController->GetBg(), rect);
	if (target_rect.width == 0 || target_rect.height == 0)
	{
		std::cout << "target_rect is not found." << std::endl;
		rect = cv::Rect();
		m_opticCount = 0;
		m_sumDelta = 0.0;
		m_speed = 0.0;
		return results;
	}

	auto frame_delta = GuiHandler::GetFrameCount() - m_startFrameCount;
	if (frame_delta % 5 == 0)
		DetectCorners(results, rect, target_rect);
	else if (!m_prevCorners.empty())
		OpticalFlow(results, rect, target_rect);
	else
	{
		std::cout << "corners are not found." << std::endl;
		rect = cv::Rect();
		m_opticCount = 0;
		m_sumDelta = 0.0;
		m_speed = 0.0;
		return results;
	}

	m_prevRect = rect;
	m_prevSubtracted = m_Subtracted.clone();

	return results;
}

std::pair<cv::Mat, cv::Rect> CornerDetector::BgSubtract(const cv::Mat& img, const cv::Mat& bg, cv::Rect& rect)
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
	target_rect = moveCenterPoint(img, local_ret, rect);
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

void CornerDetector::DetectCorners(std::vector<std::vector<Detection>>& corners_list, const cv::Rect& rect, const cv::Rect& target_rect)
{
	cv::Mat gray{};
	std::vector<Detection> corners;

	cv::cvtColor(m_Subtracted, gray, cv::COLOR_BGR2GRAY);
	cv::goodFeaturesToTrack(gray(rect), corners, 20, 0.09, 5);

	for (auto itr = corners.begin(); itr != corners.end();)
	{
		if (!target_rect.contains(*itr))
			itr = corners.erase(itr);
		else
			itr++;
	}

	corners_list.push_back(corners);
	m_prevCorners = corners;
}

void CornerDetector::OpticalFlow(std::vector<std::vector<Detection>>& corners_list, const cv::Rect& rect, const cv::Rect& target_rect)
{
	cv::Mat prev, cur;
	std::vector<Detection> area_corners, area_prev_corners;
	std::vector<Detection> good_corners, good_corners_prev;
	std::vector<uchar> status{};
	std::vector<float> err{};
	cv::Rect area(m_prevRect.x - 5, m_prevRect.y - 5, m_prevRect.width + 10, m_prevRect.height + 10);
	area |= rect;

	cv::cvtColor(m_prevSubtracted, prev, cv::COLOR_BGR2GRAY);
	cv::cvtColor(m_Subtracted, cur, cv::COLOR_BGR2GRAY);

	const auto delta = m_prevRect.tl() - area.tl();
	for (auto& corner : m_prevCorners)
		area_prev_corners.push_back(static_cast<cv::Point2f>(delta) + corner);

	cv::calcOpticalFlowPyrLK(prev(area), cur(area), area_prev_corners, area_corners, status, err, cv::Size(15, 15), 0);
	for (size_t idx = 0; idx < area_corners.size(); idx++)
	{
		if (status[idx] == 1)
		{
			good_corners.push_back(area_corners[idx] + static_cast<cv::Point2f>(area.tl()));
			good_corners_prev.push_back(area_prev_corners[idx] + static_cast<cv::Point2f>(area.tl()));
		}
	}
	CalcSpeed(good_corners_prev, good_corners, target_rect);

	for (auto itr = good_corners.begin(); itr != good_corners.end();)
	{
		*itr -= static_cast<cv::Point2f>(rect.tl());
		if (target_rect.contains(*itr))
			itr++;
		else
			itr = good_corners.erase(itr);
	}

	corners_list.push_back(good_corners);
	m_prevCorners = good_corners;
	m_opticCount++;
}

void CornerDetector::CalcSpeed(const std::vector<Detection>& prev_corners, const std::vector<Detection>& cur_corners, const cv::Rect& target_rect)
{
	double sum_delta = 0.0;
	for (size_t idx = 0; idx < cur_corners.size(); idx++)
		sum_delta += SpeedIndicator::calcDelta(prev_corners[idx], cur_corners[idx], 0.1);

	if (cur_corners.size() != 0)
		sum_delta = sum_delta / cur_corners.size();

	m_sumDelta += sum_delta;

	auto speed = SpeedIndicator::calcSpeed(m_sumDelta, m_opticCount, GuiHandler::GetFPS());
	if (GuiHandler::GetKeyEvent(static_cast<int>(' ')))
	{
		std::cout << "current_frame_count: " << GuiHandler::GetFrameCount() << std::endl;
		std::cout << "trace_time: " << m_opticCount << std::endl;
		std::cout << "answer_cul_speed: " << speed << std::endl;
	}
}