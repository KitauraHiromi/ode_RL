#include "include/world.hpp"
#define NORMAL_GRAVITY -9.8

dWorldID World::world;
dSpaceID World::space;


World::World(){
  world = dWorldCreate();       // 動力学計算世界
  space = dHashSpaceCreate(0);
  ground = dCreatePlane(space, 0, 0, 1, 0);
  contactgroup = dJointGroupCreate(0);
  contact_data = new dJointFeedback;
  
  dWorldSetGravity(world, 0, 0, NORMAL_GRAVITY);
  dWorldSetContactSurfaceLayer(world, 0.00001);
  dWorldSetContactMaxCorrectingVel ( world, 0.5 );
  //dWorldSetQuickStepNumIterations ( world, 5000 );
}

dWorldID World::GetWorldID(){
  return World::world;
}

dSpaceID World::GetSpaceID(){
return World::space;
}

World world;
