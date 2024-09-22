
#include "Arduino.h"
#include "RobotArmIK.h"
#include "Constants.h"


// - - - - - - - - - - - - - - -
// - - - - CONSTRUCTOR - - - - -
// - - - - - - - - - - - - - - -
RobotArmIK::RobotArmIK(double link1, double link2, double Base, double endeffecter, double buffer_base, double work_space, double buffer_x) {
  // Setup link length.
  _link1 = link1;
  _link2 = link2;
  _Base = Base;
  _buffer_base = buffer_base;
  _buffer_x = buffer_x;
  _endeffecter = endeffecter;
  _work_space = work_space / 2;
}

g_Code RobotArmIK::runIK(double x, double y, double z, g_Code zz) {



  double _g, base_stant, r, _x, _y;
  double _phi, _theta_base;
  double _theta;
  double _alpha, _beta, _free;


  base_stant = _Base - z;

  _y = abs((y - _work_space));

  r = sqrt((x * x) + (abs(y - _work_space) * abs(y - _work_space)));

  _theta_base = atan2(y - _work_space, x) * RadToDG;


  r = abs(r - _endeffecter);

  _theta = atan2(r, base_stant) * RadToDG;

  _g = sqrt((r * r) + (base_stant * base_stant));


  _alpha = acos(((_link1 * _link1) + (_g * _g) - (_link2 * _link2)) / (2 * _link1 * _g)) * RadToDG;



  _beta = _alpha + _theta;

  _free = acos(((_link1 * _link1) + (_link2 * _link2) - (_g * _g)) / (2 * _link1 * _link2)) * RadToDG;
  zz.x = _theta_base;
  zz.y = 180 - _beta;
  zz.z = (180 - _free - _alpha) + (180 - 90 - _theta);
  return zz;
}

g_Code RobotArmIK::runFK(double theta1, double theta2, double theta3, g_Code result) {
  double d1, d2, d3, d4;
  double _z, _x, _y;
  d1 = sin(theta1 * DGToRad) * _link1;
  d2 = cos(theta1 * DGToRad) * _link1;
  d3 = cos(theta2 * DGToRad) * _link2;
  d4 = sin(theta2 * DGToRad) * _link2;
  result.x =  cos(theta3 * DGToRad)  * (d1 + d3 + _endeffecter);;
  result.y = sin(theta3 * DGToRad) * (d1 + d3 + _endeffecter) + _work_space ;
  result.z = abs((d4 -  d2)-_Base) ;

  
  return result;
}
