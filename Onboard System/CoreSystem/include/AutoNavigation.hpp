#pragma once

#include "common.hpp"
#include "thread"
#include "list"
#include <rpc/msgpack.hpp>

namespace ControllerInterface
{

namespace AutoNavigation
{

static int path_id_counter = 1;

class Path_t
{
	int 	end_style;	// Action to perform on path completion
	float 	final_velocity;
	float 	cruise_velocity;

public:
	int id;

	MSGPACK_DEFINE_MAP(this->id, this->end_style, this->final_velocity, this->cruise_velocity);

	Path_t(float cruise_velocity, float final_velocity = 0, int end_style = 0) : cruise_velocity(cruise_velocity), final_velocity(final_velocity), end_style(end_style)
	{
		this->id = path_id_counter++;
	}

	friend void moveOnPath(Path_t &path);
	virtual int executePath() = 0;
};

class LinearPath_t : public Path_t
{
	GeoPoint_t 	start;
	GeoPoint_t 	destination;
public:
	MSGPACK_DEFINE_MAP(this->start, this->destination);

	LinearPath_t(GeoPoint_t &start, GeoPoint_t &end, float max_velocity = 10, float final_velocity = 0, int end_style = 0) : Path_t(max_velocity, final_velocity, end_style)
	{
		this->start.set(start.x, start.y, start.z);
		this->destination.set(end.x, end.y, end.z);
	}

	int executePath();
	friend Path_t *makeLinearPath(GeoPoint_t start, GeoPoint_t destination, float max_velocity);
};

class ArcPath_t : public Path_t
{
	GeoPoint_t 	focus;
	float 		radius;
	float 		arcLength;
public:
	MSGPACK_DEFINE_MAP(this->focus, this->radius, this->arcLength);

	ArcPath_t(GeoPoint_t focus, float arcLength, float radius, float max_velocity = 10, float final_velocity = 0, int end_style = 0) : Path_t(max_velocity, final_velocity, end_style)
	{
		this->focus = focus;
		this->radius = radius;
		this->arcLength = arcLength;
	}

	int executePath();
};
	

class Trajectory_t
{
	std::list<Path_t*> 	queue;
	float 				final_wait_time;

	Trajectory_t* 		next;
	std::map<int, std::list<Path_t *>::iterator> 	idPathMap;
	int id;
public:

	Trajectory_t(float final_wait_time = 0, Trajectory_t* next = nullptr);

	void 		addPath(Path_t *path);
	Path_t* 	popPath();
	int 		removePath(std::list<Path_t *>::iterator pathIterator);
	std::list<Path_t *>::iterator getPathFromID(int id);

	friend void NavigatePathQueue();
	friend void addNextTrajectory(Trajectory_t* prev, Trajectory_t* next);
};

static int 	trajectoryIDcounter;
static std::map<int, Trajectory_t*> 	idTrajectoryMap;

Path_t *makeLinearPath(GeoPoint_t start, GeoPoint_t destination, float max_velocity = 10);
void addToPathQueue(Path_t *path);
Path_t *popFromPathQueue();
int removeFromPathQueue(std::list<Path_t *>::iterator pathIterator);
std::list<Path_t *>::iterator getPathFromID(int id);
bool isNavigationInProgress();
void moveOnPath(Path_t &path);
void NavigatePathQueue();
void initialize_AutoNavigation();

void addNextTrajectory(Trajectory_t *prev, Trajectory_t *next);
Trajectory_t* getTrajectoryFromID(int id);
void addPathToTrajectory(Path_t* path, int trajectoryID);

} // namespace AutoNavigation

int enableAutoNav();
int disableAutoNav();
int getCurrentPath();
int removePath(int path_id, AutoNavigation::Trajectory_t* trajectory);
int removePathFromCurrentTrajectory(int path_id);
AutoNavigation::Trajectory_t* getCurrentTrajectory();

} // namespace ControllerInterface
