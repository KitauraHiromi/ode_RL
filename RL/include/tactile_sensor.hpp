#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <vector>
#include <cmath>
#include <iostream>

#define __USE_TACTILE__

#define SIN(x) std::sin(x)
#define COS(x) std::cos(x)
#define RAD(x) x*180./M_PI
#define DEG(x) x*M_PI/180.

typedef struct {
  dBodyID body;
  dGeomID geom;
  dVector3 value;
} MyObject;


class Tac_Sheet{
public:
  dReal center[3];
  dReal r;
  dReal range;
  int round_num;
  int axis_num;
  int total_num;
  std::vector<MyObject> tac_sensors;
  dBodyID fix_body;
  
  Tac_Sheet(dWorldID _world, dSpaceID _space, dBodyID _fix_body, dReal _cx, dReal _cy, dReal _cz, dReal _r, int _rn, int _zn);
  ~Tac_Sheet();
  void Get_Tactile_Values(std::vector<dReal> &values);
  void Set_Dist(int n, dReal dist);
  bool In_Sheet(dGeomID _geom);
  int Which_Tactile(dGeomID _geom);
  void Draw_Sheet();
  bool Callback(dGeomID o1, dGeomID o2);
private:
  dReal Convert_Tactile_Value(dReal dist);
};
