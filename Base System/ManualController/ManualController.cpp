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
#include <algorithm>

#include "../Controls/DirectControls/DirectControls.h"
//#include "../Controls/AirSimControls/AirSimControls.h"

#include "Indirect/SerialRX.h"

using namespace std;

/*
    This is Code for a physical Remote Controller for manual 
    control of the drone.
*/

class ManualController
{
    Controller *controls;
    string yaw          = "100";
    string pitch        = "200";
    string throttle     = "300";
    string roll         = "400";
    string aux1         = "500";
    string aux2         = "600";

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

    SerialRX *serial;

  protected:
  public:
    ManualController(Controller *controlobj = new DirectController(), char *portName = "/dev/ttyACM0")
    {
        controls = controlobj;
        // TODO: Add code to Calibrate the Remote data, and add a filter
        serial = new SerialRX(portName);
    }

    ~ManualController()
    {
    }

    void ExecutorSerial()
    {
        int jj = 0;
        int sz = 60;
        int scn_max = (sz/30);    // We would discard sections of data from start and end, for sanity
        int scn = 0;
        // Basically Take in values from the remote over Serial, Probably via an Arduino as middleware
        // and filter it and send it over to the API layer for Controller, to control the drone.
        while (1)
        {
            try
            {
                string pparsed;
                stringstream input_stringstream(serial->getBuff(sz));
                getline(input_stringstream, throttle, '\n'); // Discard the first entry
                scn = 0;
                while (scn < scn_max && getline(input_stringstream, pparsed, '\n'))
                {
                    try
                    {
                        ++jj;
                        ++scn;
                        // Still allow only those lines to influence which have complete sets os characters
                        if (count(pparsed.begin(), pparsed.end(), ' ') == 5)
                        {
                            string buff(pparsed);
                            cout << jj <<" Got Data [" << pparsed << "]\n";
                            stringstream input_stringstream(buff);
                            getline(input_stringstream, throttle, ' ');
                            getline(input_stringstream, yaw, ' ');
                            getline(input_stringstream, pitch, ' ');
                            getline(input_stringstream, roll, ' ');
                            getline(input_stringstream, aux1, ' ');
                            getline(input_stringstream, aux2, ' ');

                            controls->setThrottle(filter(atoi(throttle.c_str())));
                            controls->setYaw(filter(atoi(yaw.c_str())));
                            controls->setPitch(filter(atoi(pitch.c_str())));
                            controls->setRoll(filter(atoi(roll.c_str())));
                            controls->setAux1(filter(atoi(aux1.c_str())));
                            controls->setAux2(filter(atoi(aux2.c_str())));
                            sleep(0.01);
                        }
                    }
                    catch (exception &e)
                    {
                        cout<<"Error! "<<e.what();
                        break;
                    }
                }
            }
            catch (exception &e)
            {
                cout << e.what();
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
    DirectController droneControl("0.0.0.0");
    //AirSimController droneControl("0.0.0.0");
    ManualController remote(&droneControl);
    remote.ExecutorSerial();
    return 0;
}