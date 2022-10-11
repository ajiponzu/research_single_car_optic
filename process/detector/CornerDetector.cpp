#include "CornerDetector.h"
#include "../../utility/GuiHandler.h"

cv::Mat gKernel5 = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(5, 5));
cv::Mat gRoadMask = cv::imread("resources/back_kaikai.png", cv::IMREAD_GRAYSCALE);

std::vector<std::vector<Detection>> CornerDetector::Run(const cv::Mat& img, const cv::Rect& rect)
{
	std::vector<std::vector<Detection>> results{};
	const auto& frame_count = GuiHandler::GetFrameCount();

	mptr_bgController->Create(img, frame_count);
	if (frame_count < static_cast<uint64_t>(mptr_bgController->GetExHistory()))
		return results;

	auto subtract = BgSubtract(img, mptr_bgController->GetBg(), rect);
	DetectCorners(subtract, results);

	return results;
}

void CornerDetector::DetectCorners(const cv::Mat& img, std::vector<std::vector<Detection>>& corners)
{
	cv::Mat gray{};
	std::vector<Detection> corner;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::goodFeaturesToTrack(gray, corner, 20, 0.1, 10);
	corners.push_back(corner);
}

cv::Mat CornerDetector::BgSubtract(const cv::Mat& img, const cv::Mat& bg, const cv::Rect& rect)
{
	cv::Mat ret{}, mask{};
	cv::absdiff(img, bg, ret);
	cv::cvtColor(ret, ret, cv::COLOR_BGR2GRAY);
	cv::bitwise_and(ret, gRoadMask, mask);

	cv::threshold(mask(rect), mask, 0, 255, cv::THRESH_OTSU);
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, gKernel5, cv::Point(-1, -1), 2);

	cv::bitwise_and(ret(rect), mask, ret);
	cv::cvtColor(ret, ret, cv::COLOR_GRAY2BGR);

	return ret;
}
