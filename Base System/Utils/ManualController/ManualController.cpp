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
#include <fstream>
#include <thread> // std::thread
#include <algorithm>
#include <sys/stat.h>

#include "../../Controls/DirectControls/DirectControls.h"
//#include "../Controls/AirSimControls/AirSimControls.h"

//#include "Indirect/SerialRX.h"
#include "Indirect/SerialRX.h"

#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define AUX_1 0
#define AUX_2 1
#define AUX_3 2
#define AUX_4 3

#define SHOW_RC_COMMAND

float volatile delay_dump = 0;

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
        double y = (alpha * valIn) + ((1 - alpha) * avgBuffs[0]);
        avgBuffs[0] = y;
        return y;
    }

    void Reset(double alph)
    {
        alpha = alph;
    }
};

int show_debug = 0;

class ManualController
{
    string yaw = "100";
    string pitch = "200";
    string throttle = "300";
    string roll = "400";
    string aux1 = "0";
    string aux2 = "0";

    vector<double> lfactors;
    vector<double> rfactors;
    vector<double> mids;
    vector<double> mins;
    vector<double> maxs;

    int t_val;
    int p_val;
    int r_val;
    int y_val;
    int a1_val = 0;
    int a2_val = 0;
    int a3_val = 0;
    int a4_val = 0;

    bool ExecutionHold = false;

    vector<RunningAverage *> channelFilters;

    SerialRX *serial;

  protected:
  public:
    Controller *controls;

    int *auxBuffers[4] = {&a1_val, &a2_val, &a3_val, &a4_val};

    ManualController(Controller *controlobj = new DirectController(), char *portName = "/dev/ttyACM0")
    {
        controls = controlobj;
        // TODO: Add code to Calibrate the Remote data, and add a filter
        serial = new SerialRX(portName);
        for (int i = 0; i < 6; i++)
        {
            channelFilters.push_back(new RunningAverage(3, 1000, 0.45));
        }

        // Load saved Calibration Data. If file not found, Calibrate it! FileName: calibration.dat
        // Check if calibration.dat exists -->
        std::string fname = "calibration.dat";
        struct stat buffer;
        if (stat(fname.c_str(), &buffer) == 0)
        {
            std::fstream fin(fname, std::fstream::in);
            double tmp = 0;
            for (int i = 0; i < 6; i++)
            {
                fin >> tmp;
                mins.push_back(tmp);
                //cout<<tmp<<" ";
                fin >> tmp;
                maxs.push_back(tmp);
                //cout<<tmp<<" ";
                fin >> tmp;
                mids.push_back(tmp);
                //cout<<tmp<<" ";
                fin >> tmp;
                lfactors.push_back(tmp);
                //cout<<tmp<<" ";
                fin >> tmp;
                rfactors.push_back(tmp);
                //cout<<tmp<<" ";
            }
            fin.close();
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                lfactors.push_back(0);
                rfactors.push_back(0);
                mins.push_back(0);
                maxs.push_back(0);
                mids.push_back(0);
            }
            // We need to Calibrate it
            CalibrateController();
        }
        for (int i = 0; i < 6; i++)
        {
            channelFilters[i]->Reset(0.5);
        }
    }

    void CalibrateController()
    {
        /************************************************************************************************************/
        /*     Creating Moving Average Buffers, min max mid lfactor and rfactor buffers     */
        for (int i = 0; i < 6; i++)
        {
            channelFilters[i]->Reset(0.45);
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
        for (int i = 0; i < 4; i++)
        {
            if (mins[i] > maxs[i])
            {
                // Swap them
                int tmp = mins[i];
                mins[i] = maxs[i];
                maxs[i] = tmp;
            }
        }

        /* Calibration Computation */
        for (int i = 0; i < 4; i++)
        {
            if (mids[i] - mins[i] != 0)
                lfactors[i] = (127.5 / double(mids[i] - mins[i]));
            //lfactors.push_back(127.5 / double(mids[i] - mins[i]));
            else
                lfactors[i] = (1);
            //lfactors.push_back(1);
            if (maxs[i] - mids[i] != 0)
                rfactors[i] = (127.5 / double(maxs[i] - mids[i]));
            //rfactors.push_back(127.5 / double(maxs[i] - mids[i]));
            else
                rfactors[i] = (1);
            //rfactors.push_back(1);
        }
        // Save these settings
        std::fstream fin("calibration.dat", std::fstream::out);
        for (int i = 0; i < 6; i++)
        {
            fin << mins[i] << endl;
            fin << maxs[i] << endl;
            fin << mids[i] << endl;
            fin << lfactors[i] << endl;
            fin << rfactors[i] << endl;
        }
        fin.close();

        cout << "\nCalibration Completed!!!";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // After Calibration, We shall tune the Moving Average filters a bit...
        for (int i = 0; i < 6; i++)
        {
            channelFilters[i]->Reset(0.5);
        }
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
            if (ExecutionHold)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }
            parseSerialData_syncd(sz, scn_max);

            controls->setThrottle(filter(t_val, THROTTLE)); // (double(t_val - t_min) * t_factor)));
            controls->setYaw(filter(y_val, YAW));           //(double(y_val - y_min) * y_factor)));
            controls->setPitch(filter(p_val, PITCH));       // Reversed Pitch     //(double(p_val - p_min) * p_factor)));
            controls->setRoll(255 - filter(r_val, ROLL));   // Reversed Roll       //(double(r_val - r_min) * r_factor)));
            controls->setAux(1, a1_val);
            controls->setAux(2, a2_val);
            controls->setAux(3, a3_val);
            controls->setAux(4, a4_val);
            /*controls->setAux3(a2_val);
            controls->setAux4(a2_val);*/
            //std::cout<<"Help!";
            if (show_debug)
                controls->printChannels();
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
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
                        /*a1_val = atoi(aux1.c_str());
                        a2_val = atoi(aux2.c_str());*/

                        t_val = channelFilters[0]->ExpFilter(t_val);
                        y_val = channelFilters[1]->ExpFilter(y_val);
                        r_val = channelFilters[2]->ExpFilter(r_val);
                        p_val = channelFilters[3]->ExpFilter(p_val);
                    }
                }
                catch (exception &e)
                {
                    cout << "Error! PANIC!!!!" << e.what();
                    break;
                }
            }
        }
        catch (exception &e)
        {
            cout << "FUCK THIS ERROR! "<<e.what();
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
        if (vvv < 0)
            vvv = 0;
        else if (vvv > 255)
            vvv = 255;
        return int(vvv);
    }

    void StopExecutor()
    {
        ExecutionHold = false;
    }

    void ResumeExecutor()
    {
        ExecutionHold = false;
    }
};

typedef int (*func_t)(ManualController *); // function pointer
typedef int (*func_i_t)(int);              // function pointer

int channel_select = 0;
int incVal[] = {15, 130};
int PID_Controls[3][3] = {{26, 26 * 2, 26 * 3}, {26 * 4, 26 * 5, 26 * 6}, {26 * 7, 26 * 8, 26 * 9}};
//{{1100, 1200, 1300}, {1400, 1500, 1600}, {1700, 1800, 1900}};

int event_keyPlus(ManualController *obj)
{
    *(obj->auxBuffers[AUX_3]) = 255;
    return *(obj->auxBuffers[1]);
}

int event_keyMinus(ManualController *obj)
{
    *(obj->auxBuffers[AUX_3]) = 50;
    return *(obj->auxBuffers[1]);
}

int event_key_p(ManualController *obj)
{
    *(obj->auxBuffers[AUX_2]) = PID_Controls[channel_select][0];
    *(obj->auxBuffers[AUX_3]) = 127;
    return 1;
}

int event_key_i(ManualController *obj)
{
    *(obj->auxBuffers[AUX_2]) = PID_Controls[channel_select][1];
    *(obj->auxBuffers[AUX_3]) = 127;
    return 2;
}

int event_key_d(ManualController *obj)
{
    *(obj->auxBuffers[AUX_2]) = PID_Controls[channel_select][2];
    *(obj->auxBuffers[AUX_3]) = 127;
    return 2;
}

int event_key_1(ManualController *obj)
{
    //*(obj->auxBuffers[AUX_1]) = 39; //1150;
    channel_select = 0; // Do Nothing --> 1000
    *(obj->auxBuffers[AUX_2]) = 0;
    *(obj->auxBuffers[AUX_3]) = 127;
    return 1;
}

int event_key_2(ManualController *obj)
{
    //*(obj->auxBuffers[AUX_1]) = 64; //1250;
    channel_select = 1; // Do nothing --> 1349
    *(obj->auxBuffers[AUX_2]) = 0;
    *(obj->auxBuffers[AUX_3]) = 127;
    return 2;
}

int event_key_3(ManualController *obj)
{
    //*(obj->auxBuffers[AUX_1]) = 90; //1350;
    channel_select = 2;
    *(obj->auxBuffers[AUX_2]) = 0;
    *(obj->auxBuffers[AUX_3]) = 127;
    return 2;
}

int event_key_q(ManualController *obj)
{
    if (show_debug)
        show_debug = 0;
    else
        show_debug = 1;
    return 1;
}

int event_key_w(ManualController *obj)
{
    // We need to show PID Status
    return 1;
}

int event_key_e(ManualController *obj)
{
    // We need to show IMU Status
    return 1;
}

int event_key_c(ManualController *obj)
{
    // Calibrate
    obj->StopExecutor();
    obj->CalibrateController();
    obj->ResumeExecutor();
    return 1;
}

int event_key_a(ManualController *obj)
{
    *(obj->auxBuffers[AUX_1]) = 250;
    return 1;
}

int event_key_s(ManualController *obj)
{
    *(obj->auxBuffers[AUX_1]) = 0;
    return 1;
}

int event_key_h(ManualController *obj)
{
    printf("\nRAPI CALLED!!!");
    obj->controls->callRAPI(111, 0);
    return 1;
}

int event_other(ManualController *obj)
{
    *(obj->auxBuffers[0]) = 0;
    *(obj->auxBuffers[1]) = 0;
    *(obj->auxBuffers[2]) = 0;
    return 1;
}

int event_key_enter(ManualController *obj)
{
    return 1;
}

func_t KeyMap[256];

int KeyBindings_thread(ManualController *obj)
{
    while (1)
    {
        try
        {
            char key = getc(stdin);
            //printf("\t\t>>> [%d] <<<", (int)key);
            KeyMap[int(key)](obj);
        }
        catch (exception &e)
        {
            continue;
        }
    }
}

int main(int argc, char **argv)
{
    char *serialport = "/dev/ttyUSB0";
    DirectController *droneControl;
    if (argc == 1)
        droneControl = new DirectController("0.0.0.0");
    else if (argc == 2)
        droneControl = new DirectController(argv[1]);
    else if (argc == 3)
        droneControl = new DirectController(argv[1], atoi(argv[2]));
    else if (argc == 4)
    {
        droneControl = new DirectController(argv[1], atoi(argv[2]));
        serialport = argv[3];
    }

    /*
        Firstly install keybindings
    */
    for (int i = 0; i < 255; i++)
        KeyMap[i] = event_other;

    /*
        1,2,3 to select Pitch, Roll or Yaw; p, i, d to select PID param of that channel, +, - to increase/decrease values 
        q -> hide/show RC Data 
        w -> hide/show PID data 
        e -> hide/show IMU data 
        s -> disarm 
        a -> arm
    */
    KeyMap['+'] = event_keyPlus;
    KeyMap['-'] = event_keyMinus;
    KeyMap['q'] = event_key_q;
    KeyMap['w'] = event_key_w;
    KeyMap['p'] = event_key_p;
    KeyMap['i'] = event_key_i;
    KeyMap['d'] = event_key_d;
    KeyMap['1'] = event_key_1;
    KeyMap['2'] = event_key_2;
    KeyMap['3'] = event_key_3;
    KeyMap['c'] = event_key_c;
    KeyMap['a'] = event_key_a;
    KeyMap['s'] = event_key_s;
    KeyMap['h'] = event_key_h;
    KeyMap['\n'] = event_key_enter;
    KeyMap['\r'] = event_key_enter;
    ManualController remote(droneControl, serialport);
    thread KeyBindings(KeyBindings_thread, &remote);
    //ManualController remote(droneControl, "/dev/ttyUSB0");
    remote.ExecutorSerial();
    KeyBindings.join();
    return 0;
}
