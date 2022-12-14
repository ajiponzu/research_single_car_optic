#include "utility/GuiHandler.h"
#include "utility/Renderer.h"
#include "process/CarDetector.h"

int main()
{
	const int proc_imgsz = 30;
	CarDetector carDetector(L"", cv::Size(proc_imgsz, proc_imgsz));

	GuiHandler::Initialize();
	GuiHandler::SetVideoResource("resources/hiru/input.mp4");
	//GuiHandler::SetVideoResource("resources/yugata/input.mp4");
	GuiHandler::SetRenderer(carDetector.CreateRenderer());

	while (GuiHandler::EventPoll())
	{
		if (GuiHandler::MouseClickedL())
		{
			const auto& [x, y] = GuiHandler::GetClickPoint();
			cv::Point tl(std::max(0, x - proc_imgsz / 2), std::max(0, y - proc_imgsz / 2));
			cv::Point br(std::min(1920, x + proc_imgsz / 2), std::min(1080, y + proc_imgsz / 2));
			carDetector.SetRect(cv::Rect(tl, br));
		}

		if (GuiHandler::MouseClickedL() || GuiHandler::IsRunning())
		{
			const cv::Mat frame = GuiHandler::GetFrame();
			carDetector.Run(frame);
		}

		GuiHandler::Render();
	}
	cv::destroyAllWindows();
	std::cout << "end....." << std::endl;

	return 0;
}