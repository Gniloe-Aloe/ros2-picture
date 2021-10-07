#include <chrono>

#pragma once
#ifndef TIMER
#define TIMER

namespace tms {
	class timer
	{
	private:
		std::chrono::_V2::system_clock::time_point start, end, checkpoint;

	public:
		timer();

		~timer();

		float get_time_from_start();

		void set_checkpoint();

		bool check_checkpoint();

	};
}


#endif 