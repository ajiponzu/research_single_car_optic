#pragma once

struct Detection
{
	cv::Rect box{};
	float score = 0.0f;
	int class_idx = 0;
};
