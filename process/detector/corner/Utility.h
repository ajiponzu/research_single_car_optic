#pragma once

cv::Mat binarizeImg(cv::Mat& src, const int& channels = 1);
cv::Mat binarizeImgByBlock(cv::Mat& src, const int& channels = 1);
cv::Mat contrastLocalArea(cv::Mat& src);

cv::Mat getImgSlice(const cv::Mat& src, const cv::Rect& area, const int& channels = 3);
