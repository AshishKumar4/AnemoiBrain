#include "DirectControls/DirectControls.h"

#include "iostream"
#include "vector"
#include "string"
#include "stdio.h"
#include "stdlib.h"

using namespace std;

/* A Simple program to merely test the APIs */

int main()
{
    DirectController controller("0.0.0.0", 8400);
    int a = 0;
    while (1)
    {
        printf("\nEnter An Option...\n1. Throttle\n2. Yaw\n3. Pitch\n4. Roll\n:>");
        cin >> a;
        printf("\nEnter value : ");
        int val = 0;
        cin >> val;
        switch (a)
        {
        case 1:
            controller.setThrottle(val);
            break;
        case 2:
            controller.setYaw(val);
            break;
        case 3:
            controller.setPitch(val);
            break;
        case 4:
            controller.setRoll(val);
            break;
        case 5:
            controller.setAux1(val);
            break;
        case 6:
            controller.setAux2(val);
            break;
        case 0:
            exit(0);
        default:
            cout << "Incorrect Option choosen!\n";
        }
    }
    return 0;
}