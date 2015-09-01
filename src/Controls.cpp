#include "Controls.h"

Controls::Controls(void)
{
	createMainGui();
	mainGui->setVisible(false);
	createConfigGui();
	currentGui = configGui;
}

void Controls::createMainGui()
{
	saveFileName = "mainGuiSettings.xml";
	transformSources = true;

	vector<float> buffer;
	for(auto i = 0; i < 256; i++) {
		buffer.push_back(0.0);
	}
	mainGui = new ofxUICanvas();
	mainGui->addFPS();

	fpsMovingGraph = mainGui->addMovingGraph("FPS OVER TIME", buffer, 256, 0.0, 120.0);

	auto *bgSl = mainGui->addSlider("BACKGROUND",0.0,255.0,100.0);

	mainGui->addToggle("FULLSCREEN", false);
	mainGui->addToggle("TRANSFORM SOURCES", true);

	mainGui->addLabel("lDepthTresh","Depth Threshold");
	auto * dMaxSl = mainGui->addSlider("MAXDEPTH", -8, 8.0, 5.0);
	auto * dMinSl = mainGui->addSlider("MINDEPTH", -8, 8.0, -5);

	mainGui->addLabel("lResolution","Sample Resolution");
	auto * dResSl = mainGui->addSlider("RESOLUTION", .01, .5, .1);

	mainGui->addLabel("lOrgFastMesh","Meshing");
	auto * trSl = mainGui->addSlider("TRIANGLESIZE", .001, 2, 1.0);
	//trSl->setIncrement(1);

	auto * normalKSl = mainGui->addSlider("NORMAL K NEIGHBOURS", 10, 50, 20);

	auto * muSl = mainGui->addSlider("GREEDY PROJECTION MU", 1.0, 10, 2.5);

	auto * meshNeighbourSl = mainGui->addSlider("MESH MAX NEIGHBOURS", 10, 200, 150);

	vector<string> names;
	names.push_back("SOURCES");
	names.push_back("POINTS");
	names.push_back("WIREFRAME");
	names.push_back("TRIANGLEMESH");
	auto * radio = mainGui->addRadio("RENDERMODE", names, OFX_UI_ORIENTATION_VERTICAL);




	// register listener callback
	ofAddListener(mainGui->newGUIEvent, this, &Controls::guiEvent);

	// scale to fit
	mainGui->autoSizeToFitWidgets();

	// restore mainGui state from save file
	mainGui->loadSettings(saveFileName);
	// set config from mainGui state
	updateBackground(bgSl->getScaledValue());
	updateMaxDepth(dMaxSl->getScaledValue());
	updateMinDepth(dMinSl->getScaledValue());
	updateTriangleSize(trSl->getScaledValue());
	updateNormalKNeighbour(normalKSl->getScaledValue());
	updateMu(muSl->getScaledValue());
	updateMaxNearestNeighbours(meshNeighbourSl->getScaledValue());

	radio->setState(0);
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
	else if (active == "SOURCES"){
		updateRenderMode(RENDER_SOURCES);
	}

}

void Controls::createConfigGui()
{
	std::string configFileName = "configGuiSettings.xml";

	configGui = new ofxUICanvas("CAM CONFIGURATION");
	configGui->addLabel("POSITION");
	auto *slXPos = configGui->addSlider("PosX", -10.0, 10.0, 0.0);
	auto *slYPos = configGui->addSlider("PosY", -10.0, 10.0, 0.0);
	auto *slZPos = configGui->addSlider("PosZ", -10.0, 10.0, 0.0);
	configGui->addLabel("ROTATION");
	auto *slXRot = configGui->addSlider("RotX", -PI, PI, 0.0);
	auto *slYRot = configGui->addSlider("RotY", -PI, PI, 0.0);
	auto *slZRot = configGui->addSlider("RotZ", -PI, PI, 0.0);
	configGui->addSpacer(5);
	auto *btnNextCam = configGui->addButton("Next Camera", false); 
	configGui->autoSizeToFitWidgets();

	// register listener callback
	ofAddListener(configGui->newGUIEvent, this, &Controls::guiEvent);

	configGui->loadSettings(configFileName);

}

ofxUICanvas * Controls::getGui()
{
	return currentGui;
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
	else if(name == "TRANSFORM SOURCES")
	{
		auto *toggle = e.getToggle();
		transformSources = toggle->getValue();
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
	else if(name == "PosX")
	{
		auto *slider = e.getSlider();
		xPos = slider->getScaledValue();
		updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);
	}
	else if(name == "PosY")
	{
		auto *slider = e.getSlider();
		yPos = slider->getScaledValue();
		updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);
	}
	else if(name == "PosZ")
	{
		auto *slider = e.getSlider();
		zPos = slider->getScaledValue();
		updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);
	}
	else if(name == "RotX")
	{
		auto *slider = e.getSlider();
		xRot = slider->getScaledValue();
		updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);
	}
	else if(name == "RotY")
	{
		auto *slider = e.getSlider();
		yRot = slider->getScaledValue();
		updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);
	}
	else if(name == "RotZ")
	{
		auto *slider = e.getSlider();
		zRot = slider->getScaledValue();
		updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);
	}
	else if(name == "Next Camera")
	{
		auto *btn = e.getButton();
		if(btn->getValue() == true) {
			nextCamera();
		}
	}

}

void Controls::loadSettings()
{
	mainGui->loadSettings(saveFileName);
	configGui->loadSettings(configFileName);
}

void Controls::saveSettings()
{
	mainGui->saveSettings(saveFileName);
	configGui->saveSettings(configFileName);
}
