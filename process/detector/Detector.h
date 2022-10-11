#pragma once
#include "../DetectionTypes.h"

class Detector
{
private:
	friend class CarDetector;
protected:
	virtual std::vector<std::vector<Detection>>
		Run(const cv::Mat& img, const cv::Rect& rect) = 0;
};
