#pragma once
#include "Detector.h"
#include "center/BgController.h"

class CenterDetector : public Detector
{
public:
	CenterDetector()
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
	uint64_t m_trackingCount = 0;
	cv::Point2d m_prevCenter;
	cv::Point2d m_center;
	double m_sumDelta = 0.0;
	double m_speed = 0.0;

	std::pair<cv::Mat, cv::Rect> BgSubtract(const cv::Mat& img, const cv::Mat& bg, cv::Rect& rect);
	void CalcSpeed(const cv::Rect& target_rect);
};
