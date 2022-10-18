#include "CarDetector.h"
#include "detector/CornerDetector.h"
#include "../utility/GuiHandler.h"

bool g_flag = false;

CarDetector::CarDetector(const std::wstring& model_path, const cv::Size& proc_imgsz)
{
	m_ptrDetector.reset(new CornerDetector());
}

void CarDetector::Run(const cv::Mat& img)
{
	m_detectArea = m_usedDetectArea; // for rendering
	m_detections = m_ptrDetector->Run(img, m_usedDetectArea, m_resetDetect);
	m_resetDetect = false;
}

void CarDetector::ThisRenderer::Render(cv::Mat& img)
{
	cv::Mat blockImg = img(m_ptrDetector->m_detectArea);
	cv::Scalar line_color(0, 255, 0);

	DrawDetections(blockImg);
	cv::rectangle(img, m_ptrDetector->m_detectArea, cv::Scalar(255, 0, 0), 2);

	if (GuiHandler::GetFrameCount() < 500)
		line_color = cv::Scalar(0, 0, 255);

	cv::line(img, cv::Point(0, 350), cv::Point(1920, 350), line_color, 3);
	cv::line(img, cv::Point(0, 720), cv::Point(1920, 720), line_color, 3);

	//if (m_ptrDetector->m_detectArea.width > 0)
	//	g_flag = true;

	//if (g_flag)
	//	cv::imwrite(std::format("./outputs/optic_{}.png", GuiHandler::GetFrameCount()), img);
}

void CarDetector::ThisRenderer::DrawDetections(cv::Mat& img)
{
	const auto& detections = m_ptrDetector->m_detections;

	if (detections.empty())
		return;

	for (const auto& detection : detections[0])
		cv::circle(img, detection, 2, cv::Scalar(255, 0, 0), 2);
}

void CarDetector::SetRect(const cv::Rect& rect)
{
	m_detectArea = m_usedDetectArea = rect;
	m_resetDetect = true;
}