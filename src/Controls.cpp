#include "Controls.h"

Controls::Controls(void)
{
	createMainGui();
	createConfigGui();
	activateMainGui();
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
	auto * dMaxSl = mainGui->addSlider("MAXDEPTH", 0, 8.0, 5.0);
	auto * dMinSl = mainGui->addSlider("MINDEPTH", 0, 8.0, 0.0);

	mainGui->addLabel("lResolution","Sample Resolution");
	auto * dResSl = mainGui->addSlider("RESOLUTION", .01, .5, .1);

	mainGui->addLabel("lOrgFastMesh","Meshing");
	auto * trSl = mainGui->addSlider("TRIANGLESIZE", 1, 20, 5.0);
	//trSl->setIncrement(1);

	auto * normalKSl = mainGui->addSlider("NORMAL K NEIGHBOURS", 10, 50, 20);

	auto * muSl = mainGui->addSlider("GREEDY PROJECTION MU", 1.0, 10, 2.5);

	auto * meshNeighbourSl = mainGui->addSlider("MESH MAX NEIGHBOURS", 10, 200, 150);

	vector<string> names;
	names.push_back("SOURCES");
	names.push_back("POINTS");
	names.push_back("WIREFRAME");
	names.push_back("TRIANGLEMESH");
	names.push_back("TEXTURED MESH");
	auto * radio = mainGui->addRadio("RENDERMODE", names, OFX_UI_ORIENTATION_VERTICAL);

	mainGui->addSpacer(5);
	auto *btnConfig = mainGui->addButton("CALIBRATION", false); 
	auto *fovSl = mainGui->addSlider("Ad-Hoc FOV", 200.0, 400.0, 400.0 );

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
	else if (active == "TEXTURED MESH"){
		updateRenderMode(RENDER_SOURCES);
	}

}

void Controls::createConfigGui()
{
	configFileName = "configGuiSettings.xml";
	step = .01;

	configGui = new ofxUICanvas("CAM CONFIGURATION");
	configGui->addLabel("POSITION");
	auto *slXPos = configGui->addSlider("PosX", -10.0, 10.0, &xPos);
	configGui->addButton("PosX-",false);
	configGui->addButton("PosX+",false);
	auto *slYPos = configGui->addSlider("PosY", -10.0, 10.0, &yPos);
	configGui->addButton("PosY-",false);
	configGui->addButton("PosY+",false);
	auto *slZPos = configGui->addSlider("PosZ", -10.0, 10.0, &zPos);
	configGui->addButton("PosZ-",false);
	configGui->addButton("PosZ+",false);

	configGui->addLabel("ROTATION");
	auto *slXRot = configGui->addSlider("RotX", -180, 180, &xRot);
	configGui->addButton("RotX-",false);
	configGui->addButton("RotX+",false);
	auto *slYRot = configGui->addSlider("RotY", -180, 180, &yRot);
	configGui->addButton("RotY-",false);
	configGui->addButton("RotY+",false);
	auto *slZRot = configGui->addSlider("RotZ", -180, 180, &zRot);
	configGui->addButton("RotZ-",false);
	configGui->addButton("RotZ+",false);
	configGui->addSpacer(5);
	auto *btnNextCam = configGui->addButton("Next Camera", false); 
	configGui->addSpacer(5);
	auto *btnReset = configGui->addButton("Reset Calibration", false); 
	auto *btnDone = configGui->addButton("Calibration Done", false); 
	configGui->autoSizeToFitWidgets();

	// register listener callback
	ofAddListener(configGui->newGUIEvent, this, &Controls::guiEvent);

	configGui->loadSettings(configFileName);

}

void Controls::activateMainGui()
{
	mainGui->setVisible(true);
	configGui->setVisible(false);
	currentGui = mainGui;
	updateAppMode(APPMODE_RECON);
}

void Controls::activateConfigGui()
{
	mainGui->setVisible(false);
	configGui->setVisible(true);
	currentGui = configGui;
	updateAppMode(APPMODE_CONFIG);
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
		else if (active == "TEXTURED MESH"){
			updateRenderMode(RENDER_TEXTURE_MESH);
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
	else if((name == "PosX") || (name == "PosY") || (name == "PosZ")
		|| (name == "RotX") || (name == "RotY") || (name == "RotZ"))
	{
		updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);
	}
	else if(name == "PosX-") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			xPos-=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}	
	}
	else if(name == "PosX+") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			xPos+=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "PosY-") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			yPos-=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "PosY+") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			yPos+=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "PosZ-") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			zPos-=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "PosZ+") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			zPos+=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "RotX-") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			xRot-=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "RotX+") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			xRot+=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "RotY-") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			yRot-=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "RotY+") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			yRot+=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "RotZ-") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			zRot-=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot);

		}
	}
	else if(name == "RotZ+") 
	{
		auto *btn = e.getButton();
		if (btn->getValue() == true)
		{
			zRot+=step;
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot); 
		}
	}
	else if(name == "Next Camera")
	{
		auto *btn = e.getButton();
		if(btn->getValue() == true) {
			nextCamera();
		}
	}
	else if(name == "Calibration Done")
	{
		auto *btn = e.getButton();
		if(btn->getValue() == true) {
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot); 
			activateMainGui();
		}
	}
	else if(name == "Reset Calibration")
	{
		xPos = yPos = zPos = 0;
		xRot = yRot = zRot = 0;
		auto *btn = e.getButton();
		if(btn->getValue() == true) {
			updateCameraTransformation(xPos, yPos, zPos, xRot, yRot, zRot); 
		}
	}
	else if(name == "CALIBRATION")
	{
		auto *btn = e.getButton();
		if(btn->getValue() == true) {
			activateConfigGui();
		}
	}
	else if(name == "Ad-Hoc FOV")
	{
		auto *slider = e.getSlider();
		updateFov(slider->getScaledValue());
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

void Controls::setStepHigh(bool state)
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

void Controls::setCameraTransformation(float x, float y, float z, float rot_x, float rot_y, float rot_z)
{
	xPos = x;
	yPos = y;
	zPos = z;
	xRot = rot_x;
	yRot = rot_y;
	zRot = rot_z;
}