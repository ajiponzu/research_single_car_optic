#pragma once
#include "Detector.h"
#include "corner/BgController.h"

class CornerDetector : public Detector
{
public:
	CornerDetector()
	{
		mptr_bgController.reset(new BgController());
	}

	virtual std::vector<std::vector<Detection>>
		Run(const cv::Mat& img, cv::Rect& rect, const bool& reset);

private:
	std::unique_ptr<BgController> mptr_bgController;
	cv::Mat m_prevSubtracted;
	cv::Mat m_Subtracted;
	uint64_t m_startFrameCount = 0;
	cv::Rect m_prevRect;
	std::vector<Detection> m_prevCorners;

	std::pair<cv::Mat, cv::Rect> BgSubtract(const cv::Mat& img, const cv::Mat& bg, cv::Rect& rect);
	void DetectCorners(std::vector<std::vector<Detection>>& corners_list, const cv::Rect& rect, const cv::Rect& target_rect);
	void OpticalFlow(std::vector<std::vector<Detection>>& corners_list, const cv::Rect& rect, cv::Rect& target_rect);
};
