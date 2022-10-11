#pragma once
#include "Detector.h"

class CornerDetector : public Detector
{
public:
	virtual std::vector<std::vector<Detection>>
		Run(const cv::Mat& img);
};

