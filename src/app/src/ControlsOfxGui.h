#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include <boost/signals2.hpp>

#define APPMODE_CONFIG 10
#define APPMODE_RECON 20

#define RENDER_SOURCES 0
#define RENDER_POINTS 1
#define RENDER_WIRE 2
#define RENDER_MESH 3
#define RENDER_TEXTURE_MESH 4

class ControlsOfxGui
{
public:
	static ControlsOfxGui& getInstance()
	{
		static ControlsOfxGui    instance;	// Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	ofxPanel * getGui();

	void updateFramerate(float rate);

	//void guiEvent(ofxUIEventArgs &e);

	void loadSettings();
	void saveSettings();

	void setStepHigh(bool state);
	void setCameraTransformation(float x, float y, float z, float rot_x, float rot_y, float rot_z);


	boost::signals2::signal<void (float)> updateBackground;
	boost::signals2::signal<void (int)>	  updateRenderMode;
	boost::signals2::signal<void (int)>	  updateAppMode;
	boost::signals2::signal<void (float)> updateMaxDepth;
	boost::signals2::signal<void (float)> updateMinDepth;
	boost::signals2::signal<void (float)> updateTriangleSize;
	boost::signals2::signal<void (int)>   updateNormalKNeighbour;
	boost::signals2::signal<void (float)> updateMu;
	boost::signals2::signal<void (int)>   updateMaxNearestNeighbours;
	boost::signals2::signal<void (float)> updateSampleResolution;
	boost::signals2::signal<void (float, float, float, float, float, float)> updateCameraTransformation;
	boost::signals2::signal<void (void)> nextCamera;
	boost::signals2::signal<void (float)> updateFov;

	ofParameter<bool> transformSources;
private:
	ControlsOfxGui();							// Don't Implement
	ControlsOfxGui(ControlsOfxGui const&);			// Don't Implement
	void operator=(ControlsOfxGui const&);	// Don't implement

	ofxPanel *mainGui;
	ofxPanel *configGui;
	ofxPanel *currentGui;

	ofxIntSlider backgroundSlider_;
	ofParameter<int> backgroundColor;
	ofxToggle fullScreenToggle_;
	ofParameter<bool> fullScreen;
	ofxToggle transformSourcesToggle_;
	ofxFloatSlider maxDepthSlider_;
	ofParameter<float> maxDepth;
	ofxFloatSlider minDepthSlider_;
	ofParameter<float> minDepth;
	ofxFloatSlider sampleResolutionSlider_;
	ofParameter<float> sampleResolution;
	ofxIntSlider triSizeSlider_;
	ofParameter<int> triSize;
	ofxIntSlider normalKNeighbourSlider_;
	ofParameter<int> normalKNeighbour;
	ofxFloatSlider greedyProhectionMuSlider_;
	ofParameter<float> greedyProhectionMu;
	ofxIntSlider meshMaxNeighbourSlider_;
	ofParameter<int> meshMaxNeighbour;


	std::string saveFileName;
	std::string configFileName;

	void createMainGui();
	void createConfigGui();

	void activateMainGui();
	void activateConfigGui();

	void fullScreenChanged(bool &state);
	void backgroundChanged(int &color);

	float xPos, yPos, zPos, xRot, yRot, zRot;
	float step;
};

