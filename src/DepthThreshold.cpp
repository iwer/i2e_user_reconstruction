#include "DepthThreshold.h"

DepthThreshold::DepthThreshold(void)
{
	Controls::getGui()->addLabel("lDepthTresh","Depth Threshold");
	ofxUISlider * dMaxSl = Controls::getGui()->addSlider("MAXDEPTH", .1, 5.0, 100.0);
	ofxUISlider * dMinSl = Controls::getGui()->addSlider("MINDEPTH", .1, 5.0, 100.0);

	ofAddListener(Controls::getGui()->newGUIEvent, this, &DepthThreshold::guiEvent);

	Controls::postWidgetAdd();

	depthThreshMax = dMaxSl->getScaledValue();
	depthThreshMin = dMinSl->getScaledValue();
	pass_.setFilterFieldName ("z");
	pass_.setKeepOrganized(true);
}

DepthThreshold::~DepthThreshold(void)
{
	ofRemoveListener(Controls::getGui()->newGUIEvent, this, &DepthThreshold::guiEvent);
}

void DepthThreshold::processData()
{
	pass_.setFilterLimits (depthThreshMin, depthThreshMax);
	pass_.setInputCloud(inputCloud_);
	pass_.filter(*outputCloud_);
}

void DepthThreshold::guiEvent(ofxUIEventArgs &e)
{
	if(e.getName() == "MAXDEPTH")
	{
		ofxUISlider *slider = e.getSlider();
		depthThreshMax = slider->getScaledValue();
	}
	else if(e.getName() == "MINDEPTH")
	{
		ofxUISlider *slider = e.getSlider();
		depthThreshMin = slider->getScaledValue();
	}
}