#pragma once

class BgController
{
private:
	cv::Mat mBg{};
	cv::Mat mSub{};
	cv::Mat mMasks[2]{};
	std::shared_ptr<cv::BackgroundSubtractorMOG2> mptrBgExtractor;
	int mExHistory = 0;

public:
	BgController();

	void Create(const cv::Mat& frame, const int& frameCount);

	const cv::Mat& GetBg() const { return mBg; }
	const cv::Mat& GetSub() const { return mSub; }
	const int& GetExHistory() const { return mExHistory; }

	cv::Mat GetSubGray();

private:
	void CreateByMog2(const cv::Mat& frame);
	void Create(const cv::Mat& frame);

	void Create(cv::Mat& frameFloat, cv::Mat& bgFloat);
};
