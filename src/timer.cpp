#include <chrono>
#include <iostream>
#include "timer.hpp"

timer::timer()
{
	start = std::chrono::high_resolution_clock::now();
}

timer::~timer()
{
	end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<float> duration = end - start;
	std::cout << "Process duration =  " << duration.count() << " second" << std::endl;
}

float timer::get_time_from_start() {
	std::chrono::_V2::system_clock::time_point current_time;
	current_time = std::chrono::high_resolution_clock::now();

	std::chrono::duration<float> duration = current_time - start;
	return duration.count();
}