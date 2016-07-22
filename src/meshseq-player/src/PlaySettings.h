#pragma once
#include <vector>

class PlaySettings
{
public:
	int take_;
	int speed_;
	int quality_;
	bool texmap_;
	bool manual_;

	static std::vector<PlaySettings> availablePlaysettings_;

	PlaySettings();
	PlaySettings(int take, int speed, int quality, bool texmap);;
	~PlaySettings();

	static PlaySettings randomPlaysettings();
	static std::vector<PlaySettings> generateTestSequence();
};

