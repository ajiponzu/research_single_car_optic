#include "CornerDetector.h"
#include "../../utility/GuiHandler.h"

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
	cv::morphologyEx(local_ret, local_ret, cv::MORPH_OPEN, gKernel, cv::Point(-1, -1), 2);
	cv::cvtColor(local_ret, local_ret, cv::COLOR_GRAY2BGR);
}

cv::Rect moveCenterPoint(const cv::Mat& img, const cv::Mat& local_ret, cv::Rect2f& rect)
{
	cv::Rect target_rect;
	std::vector<std::vector<cv::Point>> contours_list;
	cv::findContours(local_ret, contours_list, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cv::Point correct_center_delta;
	double min_center_dist = cv::norm(cv::Point((int)rect.width, (int)rect.height));
	auto cur_center = calcRectCenter(rect) - static_cast<cv::Point>(rect.tl());

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
	rect.x = std::clamp((float)rect.x + correct_center_delta.x, 0.0f, (float)img.cols - rect.width);
	rect.y = std::clamp((float)rect.y + correct_center_delta.y, 0.0f, (float)img.rows - rect.height);

	return target_rect;
}

std::pair<cv::Mat, cv::Rect> BgSubtract(const cv::Mat& img, const cv::Mat& bg, cv::Rect2f& rect)
{
	cv::Mat ret{}, mask{};
	cv::Rect target_rect;

	cv::absdiff(img, bg, ret);
	cv::bitwise_and(ret, gRoadMask, ret);
	cv::cvtColor(ret, ret, cv::COLOR_BGR2GRAY);

	auto local_ret = ret(rect).clone();
	localBinarize(local_ret);
	cv::bitwise_and(local_ret, gRoadMask(rect), local_ret);

	cv::cvtColor(local_ret, local_ret, cv::COLOR_BGR2GRAY);
	target_rect = moveCenterPoint(img, local_ret, rect);

	local_ret = ret(rect).clone();
	localBinarize(local_ret);
	cv::bitwise_and(local_ret, gRoadMask(rect), local_ret);

	return { local_ret, target_rect };
}

std::vector<std::vector<Detection>> CornerDetector::Run(const cv::Mat& img, cv::Rect2f& rect, const bool& reset)
{
	std::vector<std::vector<Detection>> results{};
	const auto& frame_count = GuiHandler::GetFrameCount();

	mptr_bgController->Create(img, frame_count);
	if (frame_count < static_cast<uint64_t>(mptr_bgController->GetExHistory()))
		return results;

	if (rect.width == 0.0f || rect.height == 0.0f)
		return results;

	auto [subtract, target_rect] = BgSubtract(img, mptr_bgController->GetBg(), rect);
	DetectCorners(subtract, results, target_rect);

	rect.x = std::clamp(rect.x, 0.0f, (float)img.cols - rect.width);
	rect.y = std::clamp(rect.y, 0.0f, (float)img.rows - rect.height);

	return results;
}

void CornerDetector::DetectCorners(const cv::Mat& img, std::vector<std::vector<Detection>>& corners_list, const cv::Rect& target_rect)
{
	cv::Mat temp{}, gray{};
	std::vector<Detection> corners;

	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::goodFeaturesToTrack(gray, corners, 20, 0.1, 5);

	for (auto itr = corners.begin(); itr != corners.end();)
	{
		if (!target_rect.contains(*itr))
			itr = corners.erase(itr);
		else
			itr++;
	}

	corners_list.push_back(corners);
}