#pragma once

#include "ofMain.h"
#include "typedefs.h"

class AbstractPointProcessor
{
public:
	AbstractPointProcessor(void);
	virtual ~AbstractPointProcessor(void);

	void setInputCloud(CloudConstPtr);
	virtual void processData() = 0;
	CloudPtr getOutputCloud();

protected:
	CloudConstPtr inputCloud_;
	CloudPtr outputCloud_;
};

