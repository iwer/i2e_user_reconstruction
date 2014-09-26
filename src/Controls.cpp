#include "Controls.h"

Controls::Controls(void)
{
	saveFileName = "settings.xml";
	gui = new ofxUICanvas();

	vector<float> buffer;
	for(int i = 0; i < 256; i++) {
		buffer.push_back(0.0);
	}
	gui->addFPS();
	fpsMovingGraph = gui->addMovingGraph("FPS OVER TIME", buffer, 256, 0.0, 120.0);
	ofxUISlider *bgSl = gui->addSlider("BACKGROUND",0.0,255.0,100.0);
	gui->addToggle("FULLSCREEN", false);
	gui->addLabel("lDepthTresh","Depth Threshold");
	ofxUISlider * dMaxSl = gui->addSlider("MAXDEPTH", .3, 5.0, 100.0);
	ofxUISlider * dMinSl = gui->addSlider("MINDEPTH", .1, 5.0, 100.0);
	gui->addLabel("lOrgFastMesh","Meshing");
	ofxUISlider * trSl = gui->addSlider("TRIANGLESIZE", 1, 10, 100.0);
	trSl->setIncrement(1);
	vector<string> names;
	names.push_back("POINTS");
	names.push_back("WIREFRAME");
	names.push_back("TRIANGLEMESH");
	ofxUIRadio * radio = gui->addRadio("RENDERMODE", names, OFX_UI_ORIENTATION_VERTICAL);

	
	
	
	// register listener callback
	ofAddListener(gui->newGUIEvent, this, &Controls::guiEvent);

	// scale to fit
	gui->autoSizeToFitWidgets();
	// restore gui state from save file
	gui->loadSettings(saveFileName);
	// set config from gui state
	updateBackground(bgSl->getScaledValue());
	updateMaxDepth(dMaxSl->getScaledValue());
	updateMinDepth(dMinSl->getScaledValue());
	updateTriangleSize(trSl->getScaledValue());

	std::string active = radio->getActiveName();
	if(active == "POINTS"){
		updateRenderMode(RENDER_POINTS);
	}
	else if (active == "WIREFRAME"){
		updateRenderMode(RENDER_WIRE);
	}
	else if (active == "TRIANGLEMESH"){
		updateRenderMode(RENDER_MESH);
	}

}


ofxUICanvas * Controls::getGui()
{
	return gui;
}

void Controls::updateFramerate(float rate)
{
	fpsMovingGraph->addPoint(rate);
}

void Controls::guiEvent(ofxUIEventArgs &e){
	std::string name = e.getName();
	if(name == "BACKGROUND")
	{
		ofxUISlider *slider = e.getSlider();
		updateBackground(slider->getScaledValue());
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
			updateRenderMode(RENDER_POINTS);
		}
		else if (active == "WIREFRAME"){
			updateRenderMode(RENDER_WIRE);
		}
		else if (active == "TRIANGLEMESH"){
			updateRenderMode(RENDER_MESH);
		}
	}
	else if(name == "MAXDEPTH")
	{
		ofxUISlider *slider = e.getSlider();
		updateMaxDepth(slider->getScaledValue());
	}
	else if(name == "MINDEPTH")
	{
		ofxUISlider *slider = e.getSlider();
		updateMinDepth(slider->getScaledValue());
	}
	else if(name == "TRIANGLESIZE")
	{
		ofxUISlider *slider = e.getSlider();
		updateTriangleSize((int) slider->getScaledValue());
	}

}


