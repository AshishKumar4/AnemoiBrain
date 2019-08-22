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
#include <queue>

#include "Sensors/Sensors.hpp"
#include "Sensors/InertialMeasurement.hpp"
#include "Sensors/Location.hpp"

#include "ControllerInterface.hpp"

namespace ControllerInterface
{

namespace AutoNavigation
{
class Path_t
{
    GeoPoint_t start;
    GeoPoint_t destination;

    float cruise_velocity;

public:
    Path_t(GeoPoint_t &start, GeoPoint_t &end, float max_velocity = 10)
    {
        this->start.set(start.x, start.y, start.z);
        this->destination.set(end.x, end.y, end.z);
        this->cruise_velocity = max_velocity;
    }

    friend void moveOnPath(Path_t &path);
};

std::queue<Path_t*>     MainPathQueue;
std::thread*            NavigatePathQueue_thread;
std::atomic<bool>       navigationInProgress;

Path_t* makePath(GeoPoint_t start, GeoPoint_t destination, float max_velocity = 10)
{
    return new Path_t(start, destination, max_velocity);
}

void addToPathQueue(Path_t *path)
{
    MainPathQueue.push(path);
}

Path_t* popFromPathQueue()
{
    Path_t* path = MainPathQueue.front();
    MainPathQueue.pop();
    return path;
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
    moveThread = new std::thread(positionalLoop);
    moveThread->join();
    printf("\nJob Done!");
    navigationInProgress = false;
}

void NavigatePathQueue()
{
    while(1)
    {
        if(!MainPathQueue.empty())
        {
            printf("\nFound Path in Queue!");
            Path_t* path = popFromPathQueue();
            moveOnPath(*path);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

void initialize_AutoNavigation()
{
    NavigatePathQueue_thread = new std::thread(NavigatePathQueue);
}

}

}
