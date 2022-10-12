#pragma once
#include "../DetectionTypes.h"

class Detector
{
private:
	friend class CarDetector;
protected:
	virtual std::vector<std::vector<Detection>>
		Run(const cv::Mat& img, cv::Rect2f& rect, const bool& reset) = 0;
};
