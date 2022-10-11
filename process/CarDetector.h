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

		void DrawDetections(cv::Mat& img, bool label = true);
	public:
		ThisRenderer(CarDetector* ptr) : m_ptrDetector(ptr) {}
	};

	cv::Rect m_detectArea;
	std::unique_ptr<Detector> m_ptrDetector;
	std::vector<std::string> m_classNames;
	std::vector<std::vector<Detection>> m_detections;

public:
	CarDetector(const std::wstring& model_path = L"", const cv::Size& proc_imgsz = cv::Size(640, 640));

	ThisRenderer* CreateRenderer() { return new ThisRenderer(this); }

	void Run(const cv::Mat& img, const cv::Rect& rect);
};
