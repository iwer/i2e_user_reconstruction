#include "Controls.h"

std::string Controls::saveFileName = "settings.xml";
ofxUICanvas * Controls::gui = new ofxUICanvas();

Controls::Controls(void)
{

	vector<float> buffer;
	for(int i = 0; i < 256; i++) {
		buffer.push_back(0.0);
	}
	//fpsLabel =
	gui->addFPS();
	fpsMovingGraph = gui->addMovingGraph("FPS OVER TIME", buffer, 256, 0.0, 120.0);
	vector<string> names;
	names.push_back("POINTS");
	names.push_back("WIREFRAME");
	names.push_back("TRIANGLEMESH");
	ofxUIRadio * radio = gui->addRadio("RENDERMODE", names, OFX_UI_ORIENTATION_VERTICAL);

	// register listener callback
	ofAddListener(gui->newGUIEvent, this, &Controls::guiEvent);

	postWidgetAdd();

	// set rendermode from gui state
	std::string active = radio->getActiveName();
	if(active == "POINTS"){
		renderMode = RENDER_POINTS;
	}
	else if (active == "WIREFRAME"){
		renderMode = RENDER_WIRE;
	}
	else if (active == "TRIANGLEMESH"){
		renderMode = RENDER_MESH;
	}

}


Controls::~Controls(void)
{
	gui->saveSettings(saveFileName);
	delete gui;
}

ofxUICanvas * Controls::getGui()
{
	return gui;
}

void Controls::postWidgetAdd()
{
	// scale to fit
	gui->autoSizeToFitWidgets();
	// restore gui state from save file
	gui->loadSettings(saveFileName);
}

void Controls::guiEvent(ofxUIEventArgs &e){
	std::string name = e.getName();
	if(name == "BACKGROUND")
	{
		ofxUISlider *slider = e.getSlider();
		background = slider->getScaledValue();
	}
	else if(name == "FULLSCREEN")
	{
		ofxUIToggle *toggle = e.getToggle();
		ofSetFullscreen(toggle->getValue());
	}
	else if(name == "RENDERMODE")
	{
		ofxUIRadio *radio = (ofxUIRadio *) e.widget;
		std::string active = radio->getActiveName();
		if(active == "POINTS"){
			renderMode = RENDER_POINTS;
		}
		else if (active == "WIREFRAME"){
			renderMode = RENDER_WIRE;
		}
		else if (active == "TRIANGLEMESH"){
			renderMode = RENDER_MESH;
		}
	}
}


