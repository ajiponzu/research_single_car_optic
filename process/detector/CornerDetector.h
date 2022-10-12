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
		Run(const cv::Mat& img, cv::Rect2f& rect);

private:
	std::unique_ptr<BgController> mptr_bgController;
	std::vector<Detection> m_corners;

	void DetectCorners(const cv::Mat& img, std::vector<std::vector<Detection>>& corners_list, const cv::Rect& target_rect);
};
