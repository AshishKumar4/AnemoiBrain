#pragma once

#include "common.hpp"
#include "thread"
#include "list"
#include <rpc/msgpack.hpp>

enum PATH_FLAGS
{
	OVERRIDE
};

namespace ControllerInterface
{

namespace AutoNavigation
{

class Path_t
{
protected:
	int 	end_style;	// Action to perform on path completion
	float 	final_velocity;
	float 	cruise_velocity;

	int 	type;

	static int path_id_counter;

	GeoPoint_t 	destination;
public:
	int id;

	MSGPACK_DEFINE_MAP(this->id, this->end_style, this->final_velocity, this->cruise_velocity);

	Path_t(const GeoPoint_t &destination, float cruise_velocity, float final_velocity = 0, int end_style = 0) : destination(destination), cruise_velocity(cruise_velocity), final_velocity(final_velocity), end_style(end_style)
	{
		this->id = path_id_counter++;
		type = 0;
	}

	void setType(int flag)
	{
		type = flag;
	}

	friend void moveOnPath(Path_t &path);
	virtual int executePath() = 0;

	virtual void mergeIntoPath(Path_t* prev) = 0;

	friend class LinearPath_t;
	friend class ArcPath_t;
};

class LinearPath_t : public Path_t
{
	GeoPoint_t 	start;
public:
	MSGPACK_DEFINE_MAP(this->start, this->destination);

	LinearPath_t(const GeoPoint_t &start, const GeoPoint_t &end, float max_velocity = 15, float final_velocity = 0, int end_style = 0) : start(start), Path_t(end, max_velocity, final_velocity, end_style)
	{
	}

	int executePath();
	friend Path_t *makeLinearPath(const GeoPoint_t &start, const GeoPoint_t &destination, float max_velocity);
	
	void mergeIntoPath(Path_t* prev)
	{
		this->start = prev->destination;
	}
};

class ArcPath_t : public Path_t
{
	GeoPoint_t 	focus;
	float 		radius;
	float 		arcLength;
public:
	MSGPACK_DEFINE_MAP(this->focus, this->radius, this->arcLength);

	ArcPath_t(const GeoPoint_t &destination, const GeoPoint_t &focus, float arcLength, float radius, float max_velocity = 15, float final_velocity = 0, int end_style = 0) : Path_t(destination, max_velocity, final_velocity, end_style), focus(focus), arcLength(arcLength), radius(radius)
	{
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

	void 		addPath(Path_t *path, bool override = false);
	Path_t* 	popPath();
	int 		removePath(std::list<Path_t *>::iterator pathIterator);
	std::list<Path_t *>::iterator getPathFromID(int id);

	friend void NavigatePathQueue();
	friend void addNextTrajectory(Trajectory_t* prev, Trajectory_t* next);
};

Path_t*		makeLinearPath(const GeoPoint_t &start, const GeoPoint_t &destination, float max_velocity = 15);
void 		addToPathQueue(Path_t *path);
Path_t* 	popFromPathQueue();
int 		removeFromPathQueue(std::list<Path_t *>::iterator pathIterator);

std::list<Path_t *>::iterator getPathFromID(int id);

bool 		isNavigationInProgress();
void 		moveOnPath(Path_t &path);
void 		NavigatePathQueue();
void 		initialize_AutoNavigation();

void 			addNextTrajectory(Trajectory_t *prev, Trajectory_t *next);
void 			addPathToTrajectory(Path_t* path, int trajectoryID);
Trajectory_t* 	getTrajectoryFromID(int id);

void suspendCurrentPath();
} // namespace AutoNavigation

int enableAutoNav();
int disableAutoNav();
int getCurrentPath();
int removePath(int path_id, AutoNavigation::Trajectory_t* trajectory);
int removePathFromCurrentTrajectory(int path_id);
AutoNavigation::Trajectory_t* getCurrentTrajectory();

} // namespace ControllerInterface
