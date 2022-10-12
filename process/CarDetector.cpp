#include "CarDetector.h"
#include "detector/CornerDetector.h"

CarDetector::CarDetector(const std::wstring& model_path, const cv::Size& proc_imgsz)
{
	m_ptrDetector.reset(new CornerDetector());
}

void CarDetector::Run(const cv::Mat& img)
{
	m_detectArea = m_usedDetectArea; // for rendering
	m_detections = m_ptrDetector->Run(img, m_usedDetectArea);
}

void CarDetector::ThisRenderer::Render(cv::Mat& img)
{
	cv::Mat blockImg = img(m_ptrDetector->m_detectArea);
	DrawDetections(blockImg);
	cv::rectangle(img, m_ptrDetector->m_detectArea, cv::Scalar(255, 0, 0), 2);
	cv::line(img, cv::Point(0, 350), cv::Point(1920, 350), cv::Scalar(0, 255, 0), 3);
	cv::line(img, cv::Point(0, 720), cv::Point(1920, 720), cv::Scalar(0, 255, 0), 3);
}

void CarDetector::ThisRenderer::DrawDetections(cv::Mat& img)
{
	const auto& detections = m_ptrDetector->m_detections;

	if (detections.empty())
		return;

	for (const auto& detection : detections[0])
		cv::circle(img, detection, 2, cv::Scalar(255, 0, 0), 2);
}