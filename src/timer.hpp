#include <chrono>

#pragma once
#ifndef TIMER
#define TIMER

class timer
{
private:
	std::chrono::_V2::system_clock::time_point start, end;

public:
	timer();

	~timer();

	float get_time_from_start();

};

#endif 