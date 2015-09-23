#pragma once
#include <recon/AbstractSensor.h>
#include <ofxXmlSettings/src/ofxXmlSettings.h>

class SensorCalibrationSettings
{
public:
	SensorCalibrationSettings(void);
	~SensorCalibrationSettings(void);
	void saveCalibration(recon::AbstractSensor::Ptr s, int index);
	void loadCalibration(recon::AbstractSensor::Ptr s, int index);

private:
	std::string calibrationFilePrefix;
	std::string calibrationFileSuffix;

	void addExtrinsicsToSettingsFile(ofxXmlSettings &settings, recon::CameraExtrinsics::Ptr extrinsics);
	void addIntrinsicsToSettingsFile(ofxXmlSettings &settings, recon::CameraIntrinsics::Ptr intrinsics);
	recon::CameraIntrinsics::Ptr getIntrinsicsFromSettingsFile(ofxXmlSettings &settings);
	recon::CameraExtrinsics::Ptr getExtrinsicsFromSettingsFile(ofxXmlSettings &settings);
};

