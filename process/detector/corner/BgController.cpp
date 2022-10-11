#include "BgController.h"
#include "Utility.h"

BgController::BgController()
{
	mptrBgExtractor = cv::createBackgroundSubtractorMOG2();
	mExHistory = mptrBgExtractor->getHistory();
}

void BgController::Create(const cv::Mat& frame, const uint64_t& frameCount)
{
	if (frameCount < mExHistory)
		CreateByMog2(frame);
	else
		Create(frame);
}

cv::Mat BgController::GetSubGray()
{
	cv::Mat dst{};
	cv::cvtColor(mSub, dst, cv::COLOR_BGR2GRAY);
	return dst;
}

void BgController::CreateByMog2(const cv::Mat& frame)
{
	cv::Mat frameFloat{}, bgFloat{};

	frame.convertTo(frameFloat, CV_32FC3);
	if (mBg.empty())
		bgFloat = cv::Mat::zeros(cv::Size(frame.cols, frame.rows), CV_32FC3);
	else
		mBg.convertTo(bgFloat, CV_32FC3);

	mptrBgExtractor->apply(frameFloat, mSub);
	Create(frameFloat, bgFloat);
}

void BgController::Create(const cv::Mat& frame)
{
	cv::Mat frameFloat{}, bgFloat{};

	frame.convertTo(frameFloat, CV_32FC3);
	if (!mBg.empty())
		mBg.convertTo(bgFloat, CV_32FC3);

	if (mptrBgExtractor)
		mptrBgExtractor = nullptr;

	cv::absdiff(frame, mBg, mSub);
	Create(frameFloat, bgFloat);
}

void BgController::Create(cv::Mat& frameFloat, cv::Mat& bgFloat)
{
	cv::Mat carMask{};

	mMasks[1] = binarizeImg(mSub);

	if (!mMasks[0].empty())
	{
		cv::bitwise_not(mMasks[1], mMasks[1]);
		cv::bitwise_and(mMasks[1], mMasks[0], carMask);
	}
	else
		carMask = mMasks[1];

	cv::accumulateWeighted(frameFloat, bgFloat, 0.025, carMask);

	bgFloat.convertTo(mBg, CV_8UC3);
	mMasks[0] = mMasks[1];
}