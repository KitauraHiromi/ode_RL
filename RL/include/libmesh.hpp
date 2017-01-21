#include <dlfcn.h>
#include <ode/ode.h>
#include "include/StlReader.h"
#include <drawstuff/drawstuff.h>
#include <map>
#include <vector>

//static dWorldID world;
//static dSpaceID space;
//static dJointGroupID contactgroup;

void createMeshObj( dWorldID world_, dSpaceID space_, dBodyID &body_, dGeomID &geom_, dReal mass_, char* filename);
void drawMesh(dGeomID geom_);
