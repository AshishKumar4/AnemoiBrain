#include "iostream"
#include "vector"

#include "../AbstractControls/Controls.cpp"

using namespace std;

class DirectController : public Controller
{
  protected:
  public:
    DirectController()
    {
    }
    ~DirectController()
    {
    }

    /*      APIs for channel Controls       */
    void setThrottle(int val)
    {
    }

    void setPitch(int val)
    {
    }

    void setYaw(int val)
    {
    }

    void setRoll(int val)
    {
    }

    void setAux1(int val)
    {
    }
    
    void setAux2(int val)
    {
    }
};