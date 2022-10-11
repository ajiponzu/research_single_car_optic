#include "CarDetector.h"
#include "detector/CornerDetector.h"

CarDetector::CarDetector(const std::wstring& model_path, const cv::Size& proc_imgsz)
{
	m_ptrDetector.reset(new CornerDetector());
}

void CarDetector::Run(const cv::Mat& img, const cv::Rect& rect)
{
	m_detectArea = rect;
	m_detections = m_ptrDetector->Run(img);
}

void CarDetector::ThisRenderer::Render(cv::Mat& img)
{
	cv::Mat blockImg = img(m_ptrDetector->m_detectArea);
	DrawDetections(blockImg, true);
	cv::rectangle(img, m_ptrDetector->m_detectArea, cv::Scalar(255, 0, 0), 2);
	cv::line(img, cv::Point(0, 350), cv::Point(1920, 350), cv::Scalar(0, 255, 0), 3);
	cv::line(img, cv::Point(0, 720), cv::Point(1920, 720), cv::Scalar(0, 255, 0), 3);
}

void CarDetector::ThisRenderer::DrawDetections(cv::Mat& img, bool label)
{
	const auto& detections = m_ptrDetector->m_detections;
	const auto& class_names = m_ptrDetector->m_classNames;

	if (detections.empty())
		return;

	for (const auto& detection : detections[0])
	{
		const auto& box = detection.box;
		float score = detection.score;
		int class_idx = detection.class_idx;
		
		if (class_idx != 2)
			continue;

		cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);

		if (!label)
			return;

		std::stringstream ss;
		ss << std::fixed << std::setprecision(2) << score;
		std::string s = class_names[class_idx] + " " + ss.str();

		auto font_face = cv::FONT_HERSHEY_DUPLEX;
		auto font_scale = 1.0;
		int thickness = 1;
		int baseline = 0;
		auto s_size = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);
		cv::rectangle(img,
			cv::Point(box.tl().x, box.tl().y - s_size.height - 5),
			cv::Point(box.tl().x + s_size.width, box.tl().y),
			cv::Scalar(0, 0, 255), -1);
		cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5),
			font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
	}
}