#pragma once

class Renderer
{
private:
	friend class GuiHandler;
protected:
	virtual void Render(cv::Mat& img) = 0;
};
