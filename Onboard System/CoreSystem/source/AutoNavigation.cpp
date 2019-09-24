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

#include "CommonControl.hpp"
#include "AutoNavigation.hpp"

extern bool IntentionOverride;

namespace ControllerInterface
{

namespace AutoNavigation
{


int Path_t::path_id_counter = 1;

int 	trajectoryIDcounter;
std::map<int, Trajectory_t*> 	idTrajectoryMap;

Path_t *currentActivePath;
std::list<Path_t *> MainPathQueue;
std::thread *NavigatePathQueue_thread;
std::atomic<bool> navigationInProgress;
std::map<int, std::list<Path_t *>::iterator> idPathMap;
std::atomic<bool> auto_nav_toggle_flag;

Trajectory_t *currentTrajectory;
Trajectory_t *defaultTrajectory;

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -------------------------------------------- Some Important Definitions ------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

int LinearPath_t::executePath()
{
	float initial_z, final_z;
	final_z = this->destination.z;
	if (this->start.z >= this->destination.z)
	{
		initial_z = this->start.z;
	}
	else
	{
		initial_z = final_z;
	}
	printf("\n[[%f, %f]] ", initial_z, final_z);

	setFeedbackAltitude(initial_z);

	printf("\nReaching Altitude completed, hovering...");
	fflush(stdout);

	setLinearPath(this->start, this->destination, this->cruise_velocity, this->final_velocity);
	moveSavedPath(this->preWait);

	printf("\nPath completed, hovering...");
	printf("\n%f", final_z);
	fflush(stdout);

	setFeedbackAltitude(final_z);
	printf("\nDone...");
	fflush(stdout);
	return 0;
}

int ArcPath_t::executePath()
{
	return 0;
}

Path_t *makeLinearPath(const GeoPoint_t &start, const GeoPoint_t &destination, float max_velocity)
{
	Path_t *path = new LinearPath_t(start, destination, max_velocity);
	return path;
}

Trajectory_t::Trajectory_t(float final_wait_time, Trajectory_t *next) : final_wait_time(final_wait_time)
{
	this->next = next;
	this->id = trajectoryIDcounter++;
	idTrajectoryMap[id] = this;
}

void Trajectory_t::addPath(Path_t *path, bool override)
{
	if (!override || currentActivePath == nullptr)
	{
		this->queue.push_back(path);
		this->idPathMap[path->id] = this->queue.end();
	}
	else
	{
		path->setType(OVERRIDE);
		currentActivePath->mergeIntoPath(path);
		// Save old path onto the queue's front, tp be executed just after this overriden path
		this->queue.push_front(currentActivePath);
		this->queue.push_front(path);
		this->idPathMap[path->id] = this->queue.begin();
		suspendCurrentPath();
	}
}

Path_t *Trajectory_t::popPath()
{
	Path_t *path = this->queue.front();
	this->queue.pop_front();
	return path;
}

int Trajectory_t::removePath(std::list<Path_t *>::iterator pathIterator)
{
	this->queue.erase(pathIterator);
	return 0;
}

std::list<Path_t *>::iterator Trajectory_t::getPathFromID(int id)
{
	return this->idPathMap[id];
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------------- Some General Purpose APIs ------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

bool isNavigationInProgress()
{
	return navigationInProgress;
}

void moveOnPath(Path_t &path)
{
	navigationInProgress = true;
	path.executePath();
	printf("\nJob Done!");
	fflush(stdout);
	navigationInProgress = false;
}

void addNextTrajectory(Trajectory_t *prev, Trajectory_t *next)
{
	prev->next = next;
}

Trajectory_t *getTrajectoryFromID(int id)
{
	return idTrajectoryMap[id];
}

void addPathToTrajectory(Path_t *path, int trajectoryID)
{
	getTrajectoryFromID(trajectoryID)->addPath(path);
}

void NavigatePathQueue()
{
	printf("\nAuto Navigation Enabled!");
	while (1)
	{
		try
		{
			if (auto_nav_toggle_flag)
			{
				printf("\nAuto Navigation Disabled!");
				return;
			}

			while (!currentTrajectory->queue.empty())
			{
				printf("\nFound Path in Queue!");
				Path_t *path = currentTrajectory->popPath();
				currentActivePath = path;
				moveOnPath(*path);
				std::this_thread::sleep_for(std::chrono::microseconds(500));
			}
			// printf("\nTrajectory Completed!");
			// fflush(stdout);

			if (currentTrajectory->next != nullptr)
			{
				printf("\nSwitching to Next Trajectory!");
				fflush(stdout);
				currentTrajectory = currentTrajectory->next;
			}
		}
		catch (...)
		{
			printf("\nERROR!!!");
			fflush(stdout);
		}

		std::this_thread::sleep_for(std::chrono::microseconds(500));
	}
}

void initialize_AutoNavigation()
{
	auto_nav_toggle_flag = 1;
	defaultTrajectory = currentTrajectory = new Trajectory_t();
}

void suspendCurrentPath()
{

	// If Override was given, Terminate any ongoing auto navigation and execute this first
	// Then the others
	//suspendAutoNavFlag = true;
	terminateActiveFollowing();
}

} // namespace AutoNavigation

extern void switchApparentRCstream();

int enableAutoNav()
{
	if (!AutoNavigation::auto_nav_toggle_flag)
		return 1;
	AutoNavigation::auto_nav_toggle_flag = 0;
	IntentionOverride = false;
	switchApparentRCstream();
	AutoNavigation::NavigatePathQueue_thread = new std::thread(AutoNavigation::NavigatePathQueue);
	return 0;
}

int disableAutoNav()
{
	if (AutoNavigation::auto_nav_toggle_flag)
		return 1;
	AutoNavigation::auto_nav_toggle_flag = 1;
	IntentionOverride = true;
	switchApparentRCstream();
	return 0;
}

int getCurrentPath()
{
	return AutoNavigation::currentActivePath->id;
}

int removePath(int path_id, AutoNavigation::Trajectory_t *trajectory)
{
	if (path_id == AutoNavigation::currentActivePath->id)
		return 2;
	return trajectory->removePath(trajectory->getPathFromID(path_id));
}

int removePathFromCurrentTrajectory(int path_id)
{
	return removePath(path_id, AutoNavigation::currentTrajectory);
}

AutoNavigation::Trajectory_t *getCurrentTrajectory()
{
	return AutoNavigation::currentTrajectory;
}

} // namespace ControllerInterface
