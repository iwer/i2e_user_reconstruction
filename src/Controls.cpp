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
	auto * dMaxSl = gui->addSlider("MAXDEPTH", .3, 5.0, 5.0);
	auto * dMinSl = gui->addSlider("MINDEPTH", .1, 5.0, .1);

	gui->addLabel("lResolution","Sample Resolution");
	auto * dResSl = gui->addSlider("RESOLUTION", .01, .5, .1);

	gui->addLabel("lOrgFastMesh","Meshing");
	auto * trSl = gui->addSlider("TRIANGLESIZE", .001, 2, 1.0);
	//trSl->setIncrement(1);

	auto * normalKSl = gui->addSlider("NORMAL K NEIGHBOURS", 10, 50, 20);

	auto * muSl = gui->addSlider("GREEDY PROJECTION MU", .1, 10, 2.5);

	auto * meshNeighbourSl = gui->addSlider("MESH MAX NEIGHBOURS", 10, 200, 150);

	vector<string> names;
	names.push_back("SOURCES");
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
	updateNormalKNeighbour(normalKSl->getScaledValue());
	updateMu(muSl->getScaledValue());
	updateMaxNearestNeighbours(meshNeighbourSl->getScaledValue());

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
	else if (active == "SOURCES"){
		updateRenderMode(RENDER_SOURCES);
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
		auto *radio = static_cast<ofxUIRadio *>(e.widget);
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
		else if (active == "SOURCES"){
			updateRenderMode(RENDER_SOURCES);
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
	else if(name == "RESOLUTION")
	{
		auto *slider = e.getSlider();
		updateSampleResolution(slider->getScaledValue());
	}	
	else if(name == "TRIANGLESIZE")
	{
		auto *slider = e.getSlider();
		updateTriangleSize(slider->getScaledValue());
	}
	else if(name == "NORMAL K NEIGHBOURS")
	{
		auto *slider = e.getSlider();
		updateNormalKNeighbour(slider->getScaledValue());
	}
	else if(name == "GREEDY PROJECTION MU")
	{
		auto *slider = e.getSlider();
		updateNormalKNeighbour(slider->getScaledValue());
	}
	else if(name == "MESH MAX NEIGHBOURS")
	{
		auto *slider = e.getSlider();
		updateMaxNearestNeighbours(slider->getScaledValue());
	}

}

void Controls::loadSettings()
{
	gui->loadSettings(saveFileName);
}

void Controls::saveSettings()
{
	gui->saveSettings(saveFileName);
}
