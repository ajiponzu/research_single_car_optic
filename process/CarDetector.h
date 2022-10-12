#pragma once
#include "detector/Detector.h"
#include "../utility/Renderer.h"

class CarDetector
{
private:
	friend class ThisRenderer;

	class ThisRenderer : public Renderer
	{
	private:
		std::shared_ptr<CarDetector> m_ptrDetector;

		void Render(cv::Mat& img) override;

		void DrawDetections(cv::Mat& img);
	public:
		ThisRenderer(CarDetector* ptr) : m_ptrDetector(ptr) {}
	};

	bool m_resetDetect = false;
	cv::Rect2f m_detectArea, m_usedDetectArea;
	std::unique_ptr<Detector> m_ptrDetector;
	std::vector<std::string> m_classNames;
	std::vector<std::vector<Detection>> m_detections;

public:
	CarDetector(const std::wstring& model_path = L"", const cv::Size& proc_imgsz = cv::Size(640, 640));

	ThisRenderer* CreateRenderer() { return new ThisRenderer(this); }

	void Run(const cv::Mat& img);
	void SetRect(const cv::Rect& rect);
};
