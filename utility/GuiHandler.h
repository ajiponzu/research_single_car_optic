#pragma once
#include "Renderer.h"

class GuiHandler
{
private:
	static uint64_t s_frameCount;
	static bool s_isRunning;
	static bool s_wndUpdate;
	static bool s_useVideo;
	static bool s_mouseClickedL;
	static std::pair<int, int> s_clickPoint;
	static cv::Mat s_displayImg;
	static cv::Mat s_currentFrame;
	static cv::VideoCapture s_videoCapture;
	static std::unique_ptr<Renderer> s_ptrRenderer;

	// スクリーンショットを撮影する
	static void ScreenShot();

	// マウスイベントを処理する
	static void RecvMouseMsg(int event, int x, int y, int flag, void* callBack);

	// キーボード入力を処理する
	static void HandleInputKey(const int& key);

	GuiHandler() = delete;

public:
	// GUI機能を初期化する
	static void Initialize();

	// GUIイベントを処理する・アプリケーションの継続を通知する
	static bool EventPoll();

	// 毎フレーム描画する
	// (*)呼び出さないと表示ウィンドウに変更が適用されないので注意
	static void Render();

	// 経過フレームカウントを取得する
	static const auto& GetFrameCount() { return s_frameCount; }

	// 左クリックイベントの発生通知を取得する
	static const bool& MouseClickedL() { return s_mouseClickedL; }

	// 処理継続を確認する
	static const bool& IsRunning() { return s_isRunning; }

	// 現在のフレーム画像を取得する
	static cv::Mat GetFrame() { return s_currentFrame.clone(); }

	// ビデオリソースの読み込みと初期化
	static void SetVideoResource(const std::string& path);

	// レンダラーリソースの設定
	static void SetRenderer(Renderer* ptrRenderer);

	// クリックした座標を取得する
	static const std::pair<int, int>& GetClickPoint();
};