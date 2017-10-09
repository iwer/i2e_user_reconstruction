#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <mutex>

using namespace std;

#define LOGGING_ENABLED
//#define CONSOLE_LOGGING

#ifdef LOGGING_ENABLED
#ifdef CONSOLE_LOGGING
	#define LOG(x) cout
#else
	#define LOG(x) Logger::getLogger().log(x)
#endif
	#define SET_LOG_LEVEL(x) Logger::getLogger().setLogLevel(x)
#else
	#define LOG(x, y)
	#define SET_LOG_LEVEL(x)
#endif

#define LOG_DEBUG  	if (Logger::getLogger().getLogLevel() > DEBUG); \
					else LOG(DEBUG)
#define LOG_WARNING if (Logger::getLogger().getLogLevel() > WARNING); \
                    else LOG(WARNING)
#define LOG_ERROR 	LOG(ERROR)
#undef DEBUG
typedef enum {
	DEBUG,
	WARNING,
	ERROR
} LogLevel;

class Logger{
public:
	static Logger& getLogger(){
		static Logger logger;
		return logger;
	}
	ofstream& log(LogLevel l){
		_file_mutex.lock();
		_file << "[" << getLogLevelString(l) << "] " << clock() << " : ";
		_file_mutex.unlock();
		return _file;
	}
	void setLogLevel(LogLevel l){
			_level = l;
	}
	LogLevel getLogLevel(){
		return _level;
	}
	~Logger(){
		_file.close();
	}

private:
	Logger()
	 : _level(DEBUG)
	 , _file(0) {
	   _file.open("log.txt");
	}
	string getLogLevelString(LogLevel l){
		switch (l){
			case DEBUG : return "DEBUG";
			case WARNING : return "WARNING";
			case ERROR : return "ERROR";
			default : return "UNKNOWN";
		}
	}

private:
	LogLevel _level;
	ofstream _file;
	mutex _file_mutex;

};

#endif
