#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread

#include "../Controls/DirectControl/DirectControls.h"

#include "Indirect/SerialRX.h"

using namespace std;

/*
    This is Code for a physical Remote Controller for manual 
    control of the drone.
*/

class ManualController
{
    Controller *controls;
    string yaw;
    string pitch;
    string throttle;
    string roll;
    string aux1;
    string aux2;

    int max_1;
    int max_2;
    int max_3;
    int max_4;
    int max_5;
    int max_6;

    int min_1;
    int min_2;
    int min_3;
    int min_4;
    int min_5;
    int min_6;

  protected:
  public:
    ManualController(Controller* controlobj = new DirectController())
    {
        controls = controlobj;
        // TODO: Add code to Calibrate the Remote data, and add a filter 
    }

    ~ManualController()
    {

    }

    void Executor()
    {
        // Basically Take in values from the remote over Serial, Probably via an Arduino as middleware
        // and filter it and send it over to the API layer for Controller, to control the drone.
        while(1)
        {
            try 
            {
                string buff(getSerialLine());
                stringstream input_stringstream(buff);
                getline(input_stringstream, throttle, ' ');
                getline(input_stringstream, yaw, ' ');
                getline(input_stringstream, pitch, ' ');
                getline(input_stringstream, roll, ' ');
                getline(input_stringstream, aux1, ' ');
                getline(input_stringstream, aux2, ' ');

                controls->setThrottle(filter(atoi(throttle.c_str())));
                controls->setThrottle(filter(atoi(yaw.c_str())));
                controls->setThrottle(filter(atoi(pitch.c_str())));
                controls->setThrottle(filter(atoi(roll.c_str())));
                controls->setThrottle(filter(atoi(aux1.c_str())));
                controls->setThrottle(filter(atoi(aux2.c_str())));
                sleep(0.01);
            }
            catch(exception &e)
            {
                cout<<e.what();
            }
        }
    }

    int filter(int val)
    {
        return val;
    }
};

int main()
{
    DirectController control("0.0.0.0");
    ManualController remote(&control);
    remote.Executor();
    return 0;
}