#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <stdio.h>
#include <fstream>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <iostream>
#include "include/tcp.hpp"
#include "include/world.hpp"
#include "include/arm_robot3.hpp"
#include "include/nearCallback.hpp"
//#define WORLD_ID world.GetWorldID()
//#define SPACE_ID world.GetSpaceID()
// world and robot variable declaration is written in world.cpp and arm_robot*.cpp
