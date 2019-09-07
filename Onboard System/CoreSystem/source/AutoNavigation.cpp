#include "common.hpp"
#include <cstring>
#include <iostream>
#include <unistd.h>
//#include <wiringPiSPI.h>
#include <string>
#include <thread> // std::thread
#include <unistd.h>
#include <mutex>
#include <future>
#include <functional>
#include <math.h>
#include <atomic>
#include <list>
#include <map>

#include "Sensors/Sensors.hpp"
#include "Sensors/InertialMeasurement.hpp"
#include "Sensors/Location.hpp"

#include "ControllerInterface.hpp"
#include "FeedbackControl.hpp"

#include "AutoNavigation.hpp"

namespace ControllerInterface
{

namespace AutoNavigation
{

Path_t *makePath(GeoPoint_t start, GeoPoint_t destination, float max_velocity)
{
	Path_t* path = new Path_t(start, destination, max_velocity);
	return path;
}

void addToPathQueue(Path_t *path)
{
	MainPathQueue.push_back(path);
	idPathMap[path->id] = MainPathQueue.end();
}

Path_t *popFromPathQueue()
{
	Path_t *path = MainPathQueue.front();
	MainPathQueue.pop_front();
	return path;
}

int removeFromPathQueue(std::list<Path_t *>::iterator pathIterator)
{
	MainPathQueue.erase(pathIterator);
	return 0;
}

std::list<Path_t *>::iterator getPathFromID(int id)
{
	return idPathMap[id];
}

bool isNavigationInProgress()
{
	return navigationInProgress;
}

void moveOnPath(Path_t &path)
{
	navigationInProgress = true;
	setAltitude(path.destination.z);
	setLinearPath(path.start, path.destination);
	//setDestination(destination.x, destination.y, destination.z);
	moveSavedPath();
	printf("\nJob Done!");
	navigationInProgress = false;
}

void NavigatePathQueue()
{
	printf("\nAuto Navigation Enabled!");
	while (1)
	{
		if (auto_nav_toggle_flag)
		{
			printf("\nAuto Navigation Disabled!");
			return;
		}
		if (!MainPathQueue.empty())
		{
			printf("\nFound Path in Queue!");
			Path_t *path = popFromPathQueue();
			currentActivePath = path;
			moveOnPath(*path);
		}

		std::this_thread::sleep_for(std::chrono::microseconds(500));
	}
}

void initialize_AutoNavigation()
{
	auto_nav_toggle_flag = 1;
}

} // namespace AutoNavigation

int enableAutoNav()
{
	if (!AutoNavigation::auto_nav_toggle_flag)
		return 1;
	AutoNavigation::auto_nav_toggle_flag = 0;
	AutoNavigation::NavigatePathQueue_thread = new std::thread(AutoNavigation::NavigatePathQueue);
	return 0;
}

int disableAutoNav()
{
	AutoNavigation::auto_nav_toggle_flag = 1;
	return 0;
}

int getCurrentPath()
{
	return AutoNavigation::currentActivePath->id;
}

int removePath(int path_id)
{
	if(path_id == AutoNavigation::currentActivePath->id)
		return 2;
	return AutoNavigation::removeFromPathQueue(AutoNavigation::getPathFromID(path_id));
}

} // namespace ControllerInterface
