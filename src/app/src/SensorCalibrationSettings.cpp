#include "SensorCalibrationSettings.h"
#include <ofxXmlSettings.h>


SensorCalibrationSettings::SensorCalibrationSettings(void)
	: calibrationFilePrefix("calibration_sensor_")
	, calibrationFileSuffix(".xml")
{
}


SensorCalibrationSettings::~SensorCalibrationSettings(void)
{
}

void SensorCalibrationSettings::saveCalibration(recon::AbstractSensor::Ptr s, int index)
{
	if (s != nullptr) {
		auto extr = s->getDepthExtrinsics();
		auto intr = s->getDepthIntrinsics();

		ofxXmlSettings set;
		set.addValue("sensor:index", index);
		addExtrinsicsToSettingsFile(set, extr);
		addIntrinsicsToSettingsFile(set, intr);
		std::cout << "Saving calibration for sensor " << index << std::endl;
		set.saveFile(calibrationFilePrefix + std::to_string(index) + calibrationFileSuffix);
	}
}

void SensorCalibrationSettings::loadCalibration(recon::AbstractSensor::Ptr s, int index)
{
	ofxXmlSettings set;
	auto success = set.loadFile(calibrationFilePrefix + std::to_string(index) + calibrationFileSuffix);
	if (success)
	{
		auto i = set.getValue("sensor:index", -1);
		if (i == index) 
		{
			auto intr = getIntrinsicsFromSettingsFile(set);
			auto extr = getExtrinsicsFromSettingsFile(set);
			s->setDepthExtrinsics(extr);
			s->setDepthIntrinsics(intr);
			std::cout << "Loaded calibration for sensor " << index << std::endl
				<< "Extrinsics: " << std::endl << *extr << std::endl
				<< "Intrinsics: " << std::endl << *intr << std::endl;
		} else
		{
			std::cerr << "Error, calibration file seems to be intended for another sensor (requested: " << index << " loaded: " << i << ")" << std::endl;
		}
	}
	else
	{
		std::cerr << "Could not load calibration file " << calibrationFilePrefix + std::to_string(index) + calibrationFileSuffix << std::endl;
	}
}

void SensorCalibrationSettings::addIntrinsicsToSettingsFile(ofxXmlSettings &settings, recon::CameraIntrinsics::Ptr intrinsics)
{
	settings.setValue("intrinsics:focallength:x", intrinsics->getFocalLengthX());
	settings.setValue("intrinsics:focallength:y", intrinsics->getFocalLengthY());
	settings.setValue("intrinsics:principalpoint:x", intrinsics->getPrincipalPointX());
	settings.setValue("intrinsics:principalpoint:y", intrinsics->getPrincipalPointY());
	settings.setValue("intrinsics:sensor:width", intrinsics->getSensorWidth());
	settings.setValue("intrinsics:sensor:height", intrinsics->getSensorHeight());
}

recon::CameraIntrinsics::Ptr SensorCalibrationSettings::getIntrinsicsFromSettingsFile(ofxXmlSettings &settings)
{
	float foc_x =    settings.getValue("intrinsics:focallength:x", 0.0);
	float foc_y =    settings.getValue("intrinsics:focallength:y", 0.0);
	float prin_x =   settings.getValue("intrinsics:principalpoint:x", 320.0);
	float prin_y =   settings.getValue("intrinsics:principalpoint:y", 240.0);
	float s_width =  settings.getValue("intrinsics:sensor:width", 640);
	float s_height = settings.getValue("intrinsics:sensor:height", 480);

	return boost::make_shared<recon::CameraIntrinsics>(foc_x, foc_y, prin_x, prin_y, s_width, s_height);
}

recon::CameraExtrinsics::Ptr SensorCalibrationSettings::getExtrinsicsFromSettingsFile(ofxXmlSettings &settings)
{
	float rot_x = settings.getValue("extrinsics:rotation:x", 0.0);
	float rot_y = settings.getValue("extrinsics:rotation:y", 0.0);
	float rot_z = settings.getValue("extrinsics:rotation:z", 0.0);
	float rot_w = settings.getValue("extrinsics:rotation:w", 0.0);

	float trans_x = settings.getValue("extrinsics:translation:x", 0.0);
	float trans_y = settings.getValue("extrinsics:translation:y", 0.0);
	float trans_z = settings.getValue("extrinsics:translation:z", 0.0);
	float trans_w = settings.getValue("extrinsics:translation:w", 0.0);

	Eigen::Vector4f trans(trans_x, trans_y, trans_z, trans_w);
	std::cout << trans << std::endl;
	Eigen::Quaternionf rot(rot_w, rot_x, rot_y, rot_z);
	std::cout << rot.toRotationMatrix() << std::endl;
	recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(trans,	rot));
	return ext;

}

void SensorCalibrationSettings::addExtrinsicsToSettingsFile(ofxXmlSettings &settings, recon::CameraExtrinsics::Ptr extrinsics)
{
	settings.setValue("extrinsics:rotation:x", extrinsics->getRotation()->x());
	settings.setValue("extrinsics:rotation:y", extrinsics->getRotation()->y());
	settings.setValue("extrinsics:rotation:z", extrinsics->getRotation()->z());
	settings.setValue("extrinsics:rotation:w", extrinsics->getRotation()->w());

	settings.setValue("extrinsics:translation:x", extrinsics->getTranslation()->x());
	settings.setValue("extrinsics:translation:y", extrinsics->getTranslation()->y());
	settings.setValue("extrinsics:translation:z", extrinsics->getTranslation()->z());
	settings.setValue("extrinsics:translation:w", extrinsics->getTranslation()->w());
}