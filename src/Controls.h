#pragma once

#include "ofMain.h"
#include "ofxUi.h"

#define RENDER_POINTS 0
#define RENDER_WIRE 1
#define RENDER_MESH 2

class Controls
{
public:
	Controls(void);
	~Controls(void);


	static void postWidgetAdd();
	static ofxUICanvas * getGui();
	void guiEvent(ofxUIEventArgs &e);

private:
	static ofxUICanvas *gui;
	ofxUIMovingGraph * fpsMovingGraph;
	static std::string saveFileName;
	int renderMode;
	float background;


};

