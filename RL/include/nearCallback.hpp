#ifndef __NEARCALLBACK__
#define __NEARCALLBACK__

#define __NO_TACTILE__

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <stdio.h>
#include <fstream>
#include <sys/time.h>
#include <algorithm>
#include <include/world.hpp>
// for robot/arm_robot1.cpp
#ifdef __ARM_ROBOT1__
#include <include/arm_client_state_learning1.hpp>
#include <include/arm_robot1.hpp>
#endif
// for robot/arm_robot2.cpp
#ifdef __ARM_ROBOT2__
#include <include/arm_client_state_learning1.hpp>
#include <include/arm_robot2.hpp>
#endif
// for robot/arm_robot3.cpp
#ifdef __ARM_ROBOT3__
#include <include/arm_client_state_learning1.hpp>
#include <include/arm_robot3.hpp>
#endif
// for robot/baby_robot1.cpp
#ifdef __BABY_ROBOT1__
#include <include/whole_body_motion_learning1.hpp>
#include <include/baby_robot1.hpp>
#endif
// for robot/baby_robot2.cpp
#ifdef __BABY_ROBOT2__
#include <include/whole_body_motion_learning2.hpp>
#include <include/baby_robot2.hpp>
#endif
// for robot/baby_robot3.cpp
#ifdef __BABY_ROBOT3__
#include <include/whole_body_motion_learning3.hpp>
#include <include/baby_robot3.hpp>
#endif

#define WORLD_ID world.GetWorldID()
#define SPACE_ID world.GetSpaceID()
#define DT 0.01
#define softGKP 300
#define softGKD 10
#define hardGKP 3000
#define hardGKD 100

void ContactSetting(dContact* contact, double GKD, double GKP, double bounce, double bounce_vel);
void nearCallback(void *data, dGeomID o1, dGeomID o2);

#endif
