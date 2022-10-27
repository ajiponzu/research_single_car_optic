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
	uint64_t m_opticCount = 0;
	cv::Rect m_prevRect;
	std::vector<Detection> m_prevCorners;
	double m_sumDelta = 0.0;
	double m_speed = 0.0;

	std::pair<cv::Mat, cv::Rect> BgSubtract(const cv::Mat& img, const cv::Mat& bg, cv::Rect& rect);
	void DetectCorners(std::vector<std::vector<Detection>>& corners_list, const cv::Rect& rect, const cv::Rect& target_rect);
	void OpticalFlow(std::vector<std::vector<Detection>>& corners_list, const cv::Rect& rect, const cv::Rect& target_rect);
	void CalcSpeed(const std::vector<Detection>& prev_corners, const std::vector<Detection>& cur_corners, const cv::Rect& target_rect);
};
