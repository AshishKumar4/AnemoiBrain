#pragma once

#include "common.hpp"

typedef int (*func_i_t)(int);						// function pointer
typedef int (*func_vs_t)(std::vector<std::string>); // function pointer

class FlightController
{
	std::string name;
	std::string firmware;
	std::string variant;

	uintptr_t desc;

public:
	FlightController(std::string name, std::string firmware, std::string variant, uintptr_t desc) : name(name), firmware(firmware), variant(variant), desc(desc)
	{
	}

	~FlightController() 
	{
	}
};

int IssueCommand();
void Channel_Updater(int threadId);
void Raw_Init(int argc, const char *argv[]);
void sendCommand(uint8_t val, uint8_t channel);

namespace ControllerInterface
{
FlightController *MainFC;
}

void destroyFlightControllerObjs();