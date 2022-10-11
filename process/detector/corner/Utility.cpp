#include "Utility.h"

static constexpr int gBlockWid = 100;
static constexpr int gBlockHigh = 100;

cv::Mat binarizeImg(cv::Mat& src, const int& channels)
{
	cv::Mat dst{};

	if (src.channels() == 3)
		cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
	else
		dst = src;

	cv::threshold(dst, dst, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	if (channels == 3)
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);

	return dst;
}

cv::Mat binarizeImgByBlock(cv::Mat& src, const int& channels)
{
	cv::Mat dst{};

	if (src.channels() == 3)
		cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
	else
		dst = src.clone();

	for (int y = 0; y < dst.rows; y += gBlockHigh)
	{
		for (int x = 0; x < dst.cols; x += gBlockWid)
		{
			auto block = cv::Rect(x, y, std::min(gBlockWid, dst.cols - x), std::min(gBlockHigh, dst.rows - y));
			cv::Mat blockImg = dst(block);
			cv::threshold(blockImg, blockImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
		}
	}

	if (channels == 3)
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);

	return dst;
}

// 「https://www.web-dev-qa-db-ja.com/ja/c%2B%2B/%E7%94%BB%E5%83%8F%E3%81%AE%E7%B0%A1%E5%8D%98%E3%81%AA%E7%85%A7%E6%98%8E%E8%A3%9C%E6%AD%A3opencv-c/1047009394/」
cv::Mat contrastLocalArea(cv::Mat& src)
{
	cv::Mat labImg, dst;
	std::vector<cv::Mat> labs(3);
	auto clahe = cv::createCLAHE();

	cv::cvtColor(src, labImg, cv::COLOR_BGR2Lab);
	cv::split(labImg, labs);
	clahe->setClipLimit(3);
	clahe->apply(labs[0], labs[0]);
	cv::merge(labs, labImg);
	cv::cvtColor(labImg, dst, cv::COLOR_Lab2BGR);

	return dst;
}

cv::Mat getImgSlice(const cv::Mat& src, const cv::Rect& area, const int& channels)
{
	cv::Mat dst = src(area);

	if (channels == 1)
		cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);

	return dst;
}