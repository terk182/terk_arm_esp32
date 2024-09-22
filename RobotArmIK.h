#ifndef RobotArmIK_h
#define RobotArmIK_h
#include "Constants.h"
#include "Arduino.h"

struct Point2D
{
    double x;
    double y;
};

struct g_Code
{
    double x;
    double y;
    double z;
  
};



class RobotArmIK
{
  public:
    RobotArmIK(double link1, double link2, double Base,double endeffecter,double buffer_base ,double work_space,double buffer_x);
    g_Code runIK(double x, double y, double z,g_Code zz);
    g_Code runFK(double theta1, double theta2, double theta3,g_Code result);

  private:
    double _link1, _link2,  _Base ,_endeffecter,_buffer_base,_work_space,_buffer_x ;
};

#endif
