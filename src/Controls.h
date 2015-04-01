#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include <boost/signals2.hpp>

#define RENDER_SOURCES 0
#define RENDER_POINTS 1
#define RENDER_WIRE 2
#define RENDER_MESH 3

class Controls
{
public:
	static Controls& getInstance()
	{
		static Controls    instance;	// Guaranteed to be destroyed.
										// Instantiated on first use.
		return instance;
	}

	ofxUICanvas * getGui();

	void updateFramerate(float rate);


	void guiEvent(ofxUIEventArgs &e);

	void loadSettings();
	void saveSettings();

	boost::signals2::signal<void (float)> updateBackground;
	boost::signals2::signal<void (int)>	  updateRenderMode;
	boost::signals2::signal<void (float)> updateMaxDepth;
	boost::signals2::signal<void (float)> updateMinDepth;
	boost::signals2::signal<void (float)> updateTriangleSize;
	boost::signals2::signal<void (int)>   updateNormalKNeighbour;
	boost::signals2::signal<void (float)> updateMu;
	boost::signals2::signal<void (int)>   updateMaxNearestNeighbours;

private:
	Controls();							// Don't Implement
	Controls(Controls const&);			// Don't Implement
	void operator=(Controls const&);	// Don't implement

	ofxUICanvas * gui;
	ofxUIMovingGraph * fpsMovingGraph;
	std::string saveFileName;


};

