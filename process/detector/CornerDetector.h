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

	void DetectCorners(const cv::Mat& img, std::vector<std::vector<Detection>>& corners);
	cv::Mat BgSubtract(const cv::Mat& img, const cv::Mat& bg, const cv::Rect2f& rect);
};

