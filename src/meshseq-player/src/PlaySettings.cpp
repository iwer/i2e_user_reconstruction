#include "PlaySettings.h"
#include <iostream>
#include <ctime>
#include <list>
#include <algorithm>
#include <random>

std::vector<PlaySettings> PlaySettings::availablePlaysettings_;


PlaySettings::PlaySettings()
	: take_(0)
	, speed_(0)
	, quality_(0)
	, texmap_(false)
{
}

PlaySettings::PlaySettings(int take, int speed, int quality, bool texmap)
	: take_(take)
	, speed_(speed)
	, quality_(quality)
	, texmap_(texmap)
{
}


PlaySettings::~PlaySettings()
{
}

PlaySettings PlaySettings::randomPlaysettings()
{
	std::srand(std::time(nullptr));
	int i = std::rand() % availablePlaysettings_.size();
	auto s = availablePlaysettings_.at(i);
	std::cout << s.take_ << ":" << s.speed_ << ":" << s.quality_ << ":" << s.texmap_ << std::endl;
	return  s;
}

std::vector<PlaySettings> PlaySettings::generateTestSequence()
{
	std::vector<PlaySettings> setvec;
	std::srand(std::time(nullptr));
	std::vector<int> takeIdxs = { 0,1,2,3,4 };
	auto manual_take = std::rand() % 5;


	//randomize takelist
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(takeIdxs.begin(), takeIdxs.end(),g);

	int tcount = 0;
	for (auto &takeIdx : takeIdxs){

		if (takeIdx == manual_take	)
		{
			PlaySettings p(takeIdx, 0, 0, false);
			p.manual_ = true;
			setvec.push_back(p);
			std::cout << p.take_ << ":" << p.speed_ << ":" << p.quality_ << ":" << p.texmap_ << " manual" << std::endl;

		}
		else
		{
			

			auto speedIdx = (tcount >= 2) ? 1 : 0;
			auto qualIdx = tcount % 2;
			auto texmap = (std::rand() % 2) == 1 ? true : false;
			PlaySettings p(takeIdx, speedIdx, qualIdx, texmap);
			setvec.push_back(p);
			std::cout << p.take_ << ":" << p.speed_ << ":" << p.quality_ << ":" << p.texmap_ << std::endl;

			++tcount;
		}
	}
	
	return setvec;
}
