#pragma once

#include "common.hpp"

namespace ControllerInterface
{

namespace AutoNavigation
{

static int path_id_counter = 1;

class Path_t
{
	GeoPoint_t start;
	GeoPoint_t destination;

	float cruise_velocity;

public:
	int id;

	Path_t(GeoPoint_t &start, GeoPoint_t &end, float max_velocity = 10)
	{
		this->start.set(start.x, start.y, start.z);
		this->destination.set(end.x, end.y, end.z);
		this->cruise_velocity = max_velocity;
		this->id = path_id_counter++;
	}

	friend void moveOnPath(Path_t &path);
};

static Path_t* 				currentActivePath;
static std::list<Path_t *> 	MainPathQueue;
static std::thread* 			NavigatePathQueue_thread;
static std::atomic<bool> 		navigationInProgress;
static std::map<int, std::list<Path_t *>::iterator> 	idPathMap;	
static std::atomic<bool> auto_nav_toggle_flag;

Path_t *makePath(GeoPoint_t start, GeoPoint_t destination, float max_velocity = 10);
void addToPathQueue(Path_t *path);
Path_t *popFromPathQueue();
int removeFromPathQueue(std::list<Path_t *>::iterator pathIterator);
std::list<Path_t *>::iterator getPathFromID(int id);
bool isNavigationInProgress();
void moveOnPath(Path_t &path);
void NavigatePathQueue();
void initialize_AutoNavigation();
} // namespace AutoNavigation

int enableAutoNav();
int disableAutoNav();
int getCurrentPath();
int removePath(int path_id);

} // namespace ControllerInterface
