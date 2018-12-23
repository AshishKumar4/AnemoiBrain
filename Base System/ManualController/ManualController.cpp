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

//#include "Indirect/SerialRX.h"
#include "Indirect/SerialRX.h"

#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3

float volatile delay_dump=0;

using namespace std;

/*
    This is Code for a physical Remote Controller for manual 
    control of the drone.
*/

class RunningAverage
{
  private:
    vector<double> avgBuffs;
    int buffLength;
    double alpha;

  public:
    RunningAverage(int blength, int seed, double alph)
    {
        for (int j = 0; j < blength; j++)
            avgBuffs.push_back(seed);
        alpha = alph;
        buffLength = blength;
    }

    double ExpFilter(double valIn)
    {
        // an Exponential Moving Average
        double y = (alpha * valIn) + ((1-alpha)*avgBuffs[0]);
        avgBuffs[0] = y;
        return y;
    }

    void Reset(double alph)
    {
        alpha = alph;
    }
};

class ManualController
{
    Controller *controls;
    string yaw = "100";
    string pitch = "200";
    string throttle = "300";
    string roll = "400";
    string aux1 = "500";
    string aux2 = "600";

    vector<double> lfactors;
    vector<double> rfactors;
    vector<double> mids;
    vector<double> mins;
    vector<double> maxs;

    int t_val;
    int p_val;
    int r_val;
    int y_val;
    int a1_val;
    int a2_val;

    vector<RunningAverage*> channelFilters;

    SerialRX *serial;

  protected:
  public:
    ManualController(Controller *controlobj = new DirectController(), char *portName = "/dev/ttyACM0")
    {
        controls = controlobj;
        // TODO: Add code to Calibrate the Remote data, and add a filter
        serial = new SerialRX(portName);

        /************************************************************************************************************/
        /*     Creating Moving Average Buffers, min max mid lfactor and rfactor buffers     */
        for(int i = 0; i < 6; i++)
        {
            channelFilters.push_back(new RunningAverage(3, 1000, 0.45)); 
            //lfactors.push_back(0);
            //rfactors.push_back(0);
            mins.push_back(0);
            maxs.push_back(0);
            mids.push_back(0);
        }
        // When we are initializing, We set the alpha value for moving average filter as 0.5 for calibration, but we 
        // would increase it to higher value for normal usage.
        /************************************************************************************************************/

        /************************************************************************************************************/
        /*     Gathering Sample data for Calibration     */

        cout << "\n\n***Calibration in progress***\n";
        // TODO: Instead of taking simple values, take in average
        cout << "\n\tPlease push the throttle and pitch to their minimum levels within 2 seconds";
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        parseSerialData_syncd(600, 15);
        //delay1(5);
        mins[THROTTLE] = t_val;
        mins[PITCH] = p_val;
        printf("\n{%d %d}", t_val, p_val);
        cout << "\n\t\tLets hope it done rightly, Calibrated accordingly";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        cout << "\n\tPlease push the throttle and pitch to their maximum levels within 2 seconds";
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        parseSerialData_syncd(600, 15);
        //delay1(5);
        maxs[THROTTLE] = t_val;
        maxs[PITCH] = p_val;
        printf("\n{%d %d}", t_val, p_val);
        cout << "\n\t\tLets hope it done rightly, Calibrated accordingly";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        cout << "\n\tPlease push the yaw and roll to their minimum levels within 2 seconds";
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        parseSerialData_syncd(600, 15);
        //delay1(5);
        mins[ROLL] = r_val;
        mins[YAW] = y_val;
        printf("\n{%d %d}", r_val, y_val);
        cout << "\n\t\tLets hope it done rightly, Calibrated accordingly";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        cout << "\n\tPlease push the yaw and roll to their maximum levels within 2 seconds";
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        parseSerialData_syncd(600, 15);
        //delay1(5);
        maxs[ROLL] = r_val;
        maxs[YAW] = y_val;
        printf("\n{%d %d}", r_val, y_val);
        cout << "\n\t\tLets hope it done rightly, Calibrated accordingly";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        cout << "\n\tPlease leave the throttle and place them in the middle positions within 2 seconds";
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        parseSerialData_syncd(600, 15);
        //delay1(5);
        mids[THROTTLE] = t_val;
        mids[PITCH] = p_val;
        mids[ROLL] = r_val;
        mids[YAW] = y_val;
        printf("\n{%d %d %d %d}", t_val, p_val, r_val, y_val);

        /* In case you just decided to switch your controls to be mapped in opposite ways -_- */
        for(int i = 0; i < 4; i++)
        {
            if(mins[i] > maxs[i])
            {
                // Swap them 
                int tmp = mins[i];
                mins[i] = maxs[i];
                maxs[i] = tmp;
            }
        }

        /* Calibration Computation */
        for(int i = 0; i < 4; i++)
        {
            if(mids[i] - mins[i] != 0)
                lfactors.push_back(127.5 / double(mids[i] - mins[i]));
            else 
                lfactors.push_back(1);
            if(maxs[i] - mids[i] != 0)
                rfactors.push_back(127.5 / double(maxs[i] - mids[i]));
            else 
                rfactors.push_back(1);
        }

        cout << "\nCalibration Completed!!!";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // After Calibration, We shall tune the Moving Average filters a bit...
        for(int i = 0; i < 6; i++)
        {
            channelFilters[i]->Reset(0.5);
        }

        printf("\n\nCalibration Parameters --> ");
        //cout<<"\nRange: "<<t_range<<" "<<r_range<<" "<<y_range<<" "<<p_range<<"\n";
        //cout<<"\nFactors: "<<t_lfactor<<" "<<r_lfactor<<" "<<y_lfactor<<" "<<p_lfactor<<"\n";
        /************************************************************************************************************/
    }

    ~ManualController()
    {
    }

    void ExecutorSerial()
    {
        int jj = 0;
        int sz = 90;
        int scn_max = 1; //(sz / 30); // We would discard sections of data from start and end, for sanity
        // Basically Take in values from the remote over Serial, Probably via an Arduino as middleware
        // and filter it and send it over to the API layer for Controller, to control the drone.
        
        //serial->openSerial();
        while (1)
        {
            parseSerialData_syncd(sz, scn_max);

            cout << "Data: ";

            controls->setThrottle(filter(t_val, THROTTLE)); // (double(t_val - t_min) * t_factor)));
            controls->setYaw(filter(y_val, YAW));           //(double(y_val - y_min) * y_factor)));
            controls->setPitch(255 - filter(p_val, PITCH));   // Reversed Pitch     //(double(p_val - p_min) * p_factor)));
            controls->setRoll(255 - filter(r_val, ROLL));  // Reversed Roll       //(double(r_val - r_min) * r_factor)));
            //controls->setAux1((a1_val));
            //controls->setAux2((a2_val));
            cout << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        //serial->closeSerial();
    }

    void parseSerialData_syncd(int sz, int scn_max)
    {
        try
        {
            string pparsed;
            stringstream input_stringstream(serial->getBuff_synced(sz));
            getline(input_stringstream, throttle, '\n'); // Discard the first entry
            int scn = 0;
            while (scn < scn_max && getline(input_stringstream, pparsed, '\n'))
            {
                try
                {
                    // Still allow only those lines to influence which have complete sets of characters
                    if (count(pparsed.begin(), pparsed.end(), ' ') == 5)
                    {
                        ++scn;
                        string buff(pparsed);
                        //cout << " Got Data [" << pparsed << "]\n";
                        stringstream input_stringstream(buff);
                        getline(input_stringstream, throttle, ' ');
                        getline(input_stringstream, yaw, ' ');
                        getline(input_stringstream, pitch, ' ');
                        getline(input_stringstream, roll, ' ');
                        getline(input_stringstream, aux1, ' ');
                        getline(input_stringstream, aux2, ' ');

                        t_val = atoi(throttle.c_str());
                        y_val = atoi(yaw.c_str());
                        r_val = atoi(roll.c_str());
                        p_val = atoi(pitch.c_str());
                        a1_val = atoi(aux1.c_str());
                        a2_val = atoi(aux2.c_str());

                        t_val = channelFilters[0]->ExpFilter(t_val);
                        y_val = channelFilters[1]->ExpFilter(y_val);
                        r_val = channelFilters[2]->ExpFilter(r_val);
                        p_val = channelFilters[3]->ExpFilter(p_val);
                    }
                }
                catch (exception &e)
                {
                    cout << "Error! " << e.what();
                    break;
                }
            }
        }
        catch (exception &e)
        {
            cout << e.what();
        }
    }

    void parseSerialData(int sz, int scn_max)
    {
        try
        {
            string pparsed;
            stringstream input_stringstream(serial->getBuff(sz));
            getline(input_stringstream, throttle, '\n'); // Discard the first entry
            int scn = 0;
            while (scn < scn_max && getline(input_stringstream, pparsed, '\n'))
            {
                try
                {
                    // Still allow only those lines to influence which have complete sets os characters
                    if (count(pparsed.begin(), pparsed.end(), ' ') == 5)
                    {
                        ++scn;
                        string buff(pparsed);
                        //cout << " Got Data [" << pparsed << "]\n";
                        stringstream input_stringstream(buff);
                        getline(input_stringstream, throttle, ' ');
                        getline(input_stringstream, yaw, ' ');
                        getline(input_stringstream, pitch, ' ');
                        getline(input_stringstream, roll, ' ');
                        getline(input_stringstream, aux1, ' ');
                        getline(input_stringstream, aux2, ' ');

                        t_val = atoi(throttle.c_str());
                        y_val = atoi(yaw.c_str());
                        r_val = atoi(roll.c_str());
                        p_val = atoi(pitch.c_str());
                        a1_val = atoi(aux1.c_str());
                        a2_val = atoi(aux2.c_str());

                        t_val = channelFilters[0]->ExpFilter(t_val);
                        y_val = channelFilters[1]->ExpFilter(y_val);
                        r_val = channelFilters[2]->ExpFilter(r_val);
                        p_val = channelFilters[3]->ExpFilter(p_val);

                        /*controls->setThrottle(filter(atoi(throttle.c_str())));
                        controls->setYaw(filter(atoi(yaw.c_str())));
                        controls->setPitch(filter(atoi(pitch.c_str())));
                        controls->setRoll(filter(atoi(roll.c_str())));
                        controls->setAux1(filter(atoi(aux1.c_str())));
                        controls->setAux2(filter(atoi(aux2.c_str())));*/
                        //sleep(0.01);
                    }
                }
                catch (exception &e)
                {
                    cout << "Error! " << e.what();
                    break;
                }
            }
        }
        catch (exception &e)
        {
            cout << e.what();
        }
    }

    int filter(int val, int channel)
    {
        double vvv = double(val);
        /*if(vvv < lfactors[channel]) vvv = lfactors[channel];
        else if(vvv > rfactors[channel]) vvv = rfactors[channel];
        cout<<"["<<val<<"_";*/
        // Eq --> ((no-ni)/(bo-bi))*(a-bi) + ni; (no-ni)*(bo-bi) is our factor
        // We take into account the mid stick values
        if (vvv <= mids[channel])
        {
            vvv = ((vvv - mins[channel]) * lfactors[channel]); //mins[channel];
        }
        else
        {
            vvv = ((vvv - mids[channel]) * rfactors[channel]) + 127.5; // + mids[channel];
        }
        if(vvv < 0) vvv = 0;
        else if(vvv > 255) vvv = 255;
        cout << "[" << int(vvv) << " ]--";
        return int(vvv);
    }
};

int main(int argc, char **argv)
{
    DirectController* droneControl;
    if(argc == 1)
        droneControl = new DirectController("0.0.0.0");
    else if(argc == 2)
        droneControl = new DirectController(argv[1]);
    else if(argc == 3)
        droneControl = new DirectController(argv[1], atoi(argv[2]));

    //DirectController droneControl("0.0.0.0");
    ManualController remote(droneControl, "/dev/ttyUSB0");
    remote.ExecutorSerial();
    return 0;
}
