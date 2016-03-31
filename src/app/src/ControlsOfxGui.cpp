#include "ControlsOfxGui.h"

ControlsOfxGui::ControlsOfxGui()
	: transformSources("Transform Sources", true)
	, backgroundColor("BACKGROUND", 100, 0, 255)
	, fullScreen("FullScreen", false)
	, maxDepth("MaxDepth", 5, 0, 8)
	, minDepth("Mindepth", 0, 0, 8)
	, sampleResolution("SampleResolution", .1, .01, .5)
	, triSize("TriangleSize", 5, 1, 20)
	, normalKNeighbour("NormalKNeighbours", 20, 10, 50)
	, greedyProhectionMu("GreedyProjectionMu", 2.5, 1, 10)
	, meshMaxNeighbour("MeshMaxNeighbours", 150, 10, 200)
{
	createMainGui();
	createConfigGui();
	activateMainGui();


}

void ControlsOfxGui::loadSettings()
{
	mainGui->loadFromFile(saveFileName);
	configGui->loadFromFile(configFileName);
}

void ControlsOfxGui::saveSettings()
{
	if (mainGui != nullptr) {
		mainGui->saveToFile(saveFileName);
	}
	if (configGui != nullptr) {
		configGui->saveToFile(configFileName);
	}
}

void ControlsOfxGui::setStepHigh(bool state)
{
	if(state)
	{
		step=.1;
	}
	else
	{
		step = .01;
	}
}

void ControlsOfxGui::setCameraTransformation(float x, float y, float z, float rot_x, float rot_y, float rot_z)
{
	xPos = x;
	yPos = y;
	zPos = z;
	xRot = rot_x;
	yRot = rot_y;
	zRot = rot_z;
}

void ControlsOfxGui::createMainGui()
{
	saveFileName = "mainGuiSettings.xml";
	transformSources = true;
	mainGui = new ofxPanel();
	mainGui->setup();


	backgroundColor.addListener(this, &ControlsOfxGui::backgroundChanged);
	fullScreen.addListener(this, &ControlsOfxGui::fullScreenChanged);

	mainGui->add(backgroundSlider_.setup(backgroundColor));
	mainGui->add(fullScreenToggle_.setup(fullScreen));
	mainGui->add(transformSourcesToggle_.setup(transformSources));
	mainGui->add(minDepthSlider_.setup(minDepth));
	mainGui->add(maxDepthSlider_.setup(maxDepth));
	mainGui->add(sampleResolutionSlider_.setup(sampleResolution));
	mainGui->add(triSizeSlider_.setup(triSize));
	mainGui->add(normalKNeighbourSlider_.setup(normalKNeighbour));
	mainGui->add(greedyProhectionMuSlider_.setup(greedyProhectionMu));
	mainGui->add(meshMaxNeighbourSlider_.setup(meshMaxNeighbour));

}

void ControlsOfxGui::createConfigGui()
{
}

void ControlsOfxGui::activateMainGui()
{
	currentGui = mainGui;
	updateAppMode(APPMODE_RECON);
}

void ControlsOfxGui::activateConfigGui()
{
	currentGui = configGui;
	updateAppMode(APPMODE_CONFIG);
}

void ControlsOfxGui::fullScreenChanged(bool &state)
{
	ofSetFullscreen(state);
}

void ControlsOfxGui::backgroundChanged(int &color)
{
	updateBackground(color);
}

ofxPanel* ControlsOfxGui::getGui()
{
	return currentGui;
}

void ControlsOfxGui::updateFramerate(float rate)
{
}