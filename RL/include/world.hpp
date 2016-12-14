#ifndef __WORLD__
#define __WORLD__

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

class World{
public:
  static dWorldID world;       // 動力学計算世界
  static dSpaceID space;
  dGeomID  ground;
  dJointGroupID contactgroup;
  dJointFeedback* contact_data;
  
  World();
  dWorldID GetWorldID();
  dSpaceID GetSpaceID();
};

extern World world;
#endif
