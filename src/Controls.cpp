#include "Controls.h"

Controls::Controls(void)
{
	saveFileName = "settings.xml";
	gui = new ofxUICanvas();

	vector<float> buffer;
	for(auto i = 0; i < 256; i++) {
		buffer.push_back(0.0);
	}
	gui->addFPS();
	fpsMovingGraph = gui->addMovingGraph("FPS OVER TIME", buffer, 256, 0.0, 120.0);
	auto *bgSl = gui->addSlider("BACKGROUND",0.0,255.0,100.0);
	gui->addToggle("FULLSCREEN", false);
	gui->addLabel("lDepthTresh","Depth Threshold");
	auto * dMaxSl = gui->addSlider("MAXDEPTH", .3, 5.0, 100.0);
	auto * dMinSl = gui->addSlider("MINDEPTH", .1, 5.0, 100.0);
	gui->addLabel("lOrgFastMesh","Meshing");
	auto * trSl = gui->addSlider("TRIANGLESIZE", .01, 10, 100.0);
	//trSl->setIncrement(1);
	vector<string> names;
	names.push_back("POINTS");
	names.push_back("WIREFRAME");
	names.push_back("TRIANGLEMESH");
	auto * radio = gui->addRadio("RENDERMODE", names, OFX_UI_ORIENTATION_VERTICAL);




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
	std::cout << "Active radio: " << radio->getValue() << std::endl;
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
	auto name = e.getName();
	if(name == "BACKGROUND")
	{
		auto *slider = e.getSlider();
		updateBackground(slider->getScaledValue());
	}
	else if(name == "FULLSCREEN")
	{
		auto *toggle = e.getToggle();
		ofSetFullscreen(toggle->getValue());
	}
	else if(name == "RENDERMODE")
	{
		auto *radio = (ofxUIRadio *) e.widget;
		auto active = radio->getActiveName();
		std::cout << "Active radio: " << radio->getValue() << std::endl;
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
		auto *slider = e.getSlider();
		updateMaxDepth(slider->getScaledValue());
	}
	else if(name == "MINDEPTH")
	{
		auto *slider = e.getSlider();
		updateMinDepth(slider->getScaledValue());
	}
	else if(name == "TRIANGLESIZE")
	{
		auto *slider = e.getSlider();
		updateTriangleSize(slider->getScaledValue());
	}

}


