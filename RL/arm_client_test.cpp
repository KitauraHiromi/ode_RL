#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#endif

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <stdio.h>
#include <fstream>
#include <sys/time.h>
#include "include/tcp.hpp"
#include "include/world.hpp"
#include "include/arm_robot3.hpp"
#include "include/nearCallback.hpp"
#define WORLD_ID world.GetWorldID()
#define SPACE_ID world.GetSpaceID()
#define NORMAL_GRAVITY -9.8
#define ONE_ITER_LENGTH 10000
#define SIGNAL_SPAN 100

#define __DEBUG__ 1

// world and robot variable declaration is written in world.cpp and arm_robot*.cpp

bool VIEW = false;
bool ITER_FINISH = false;
int count, sum_count;

#if __DEBUG__
// file for debug
std::ofstream fout("socket.log");
struct timeval start_time, end_time;
#endif

// TCP/IP communication
Client client = Client();

void restart(Arm_Robot3* robot){
  client.Create_connection();
  char SIGNAL[] = "RESTART_SIGNAL";
  client.Send(SIGNAL, sizeof(SIGNAL));
  client.Close_connection();

  // simulation reset. maybe including some bugs.
  delete robot;
  robot = new Arm_Robot3(WORLD_ID, SPACE_ID);
  printf("\nRESTART\n");
  count = 0;
}

void command(int cmd) { /***  Recieve key input ***/
  switch (cmd) {
  case 'r': restart(robot); break; 
  }
}

void start(){
  // Initialize drawing API
  float xyz[3] = {  3.04, 1.28, 0.76 };
  float hpr[3] = {-160.0, 4.50, 0.00 };
  dsSetViewpoint(xyz, hpr);
  dWorldSetGravity(WORLD_ID, 0, 0, NORMAL_GRAVITY);
}


void simLoop(int pause) {
  // contact_data_relative initializing
  world.contact_data_relative[0] = 0;
  world.contact_data_relative[1] = 0;
  world.contact_data_relative[2] = 0;
  
  // Call dSpaceCollide in the begining of simLoop.
  // dSpaceCollide calls nearCallback.
  dSpaceCollide(SPACE_ID, 0, &nearCallback); 
  robot->control();
  dWorldStep(WORLD_ID, 0.01);
  dJointGroupEmpty(world.contactgroup);
  
  // TCP/IP communication
  // communication protcol should be written in config file. 
  if(count % SIGNAL_SPAN == 0){
    // description
    char recieve_buf[256]; char send_buf[256];
    memset(send_buf, ' ', sizeof(send_buf));
    robot->ANGLE[1] = dJointGetHingeAngle(robot->joint[1]) * 180 / M_PI;
    robot->ANGLE[2] = dJointGetHingeAngle(robot->joint[2]) * 180 / M_PI;
    robot->ANGLE[3] = dJointGetHingeAngle(robot->joint[3]) * 180 / M_PI;
    sprintf(send_buf, "%d,%lf,%lf,%lf,%lf",
	    count,
	    robot->ANGLE[1],
	    robot->ANGLE[2],
	    robot->ANGLE[3],
	    world.contact_data_relative[2] );

    bool TCP_SUCCEED = false;
    int n = -1;
    while(!TCP_SUCCEED){
      client.Create_connection();
      client.Send(send_buf, sizeof(send_buf));
      n = client.Recieve(recieve_buf, sizeof(recieve_buf));
      if(n != -1) TCP_SUCCEED = true;
      client.Close_connection();
    }

    printf("send %s recieve %.12s rec_num %d count %d \n",send_buf, recieve_buf, n, count);
    double angle[NUM-1];
    sscanf(recieve_buf, "%lf %lf %lf", &angle[0], &angle[1], &angle[2]);

#if __DEBUG__
    gettimeofday(&end_time, NULL);
    fout << (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start.tv_usec)*1.0E-6 << ' '
	 << sum_count << ' '
	 << count << ' '
	 << n << std::endl;
#endif
    
    // Robot move
    robot->ANGLE[1] = angle[0];
    robot->ANGLE[2] = angle[1];
    robot->ANGLE[3] = angle[2];
  }
  
  // Drawing robot
  if(VIEW){
    dsSetColor(1.0,1.0,1.0);
    for (int i = 0; i <NUM; i++ )
      dsDrawCapsule(dBodyGetPosition(robot->link[i]),
		    dBodyGetRotation(robot->link[i]),
		    robot->l[i], robot->r[i]);
    dsSetColor(1.0, 0.0, 0.0);
    dsDrawBox(dBodyGetPosition(robot->tac_sensor.body),
	      dBodyGetRotation(robot->tac_sensor.body),
	      robot->tac_size);
  }

  // counter increment
  ++sum_count;
  if(++count > ONE_ITER_LENGTH){
    ITER_FINISH = true;
    count = 0;
  }  
}

int main(int argc, char *argv[]) {
  dsFunctions fn; // drawing function

#if __DEBUG__
  gettimeofday(&start_time, NULL);
#endif
  
  if(argc > 1){
    VIEW = true;
    printf("%s\n", argv[1]);
  }
  
  if(VIEW){
    fn.version = DS_VERSION;   fn.start = &start;
    fn.step   = &simLoop;      fn.command = &command;
    fn.path_to_textures = "../../drawstuff/textures";
  }
  
  while(1){
    // Initialization
    dInitODE();
    ITER_FINISH = false;
    world = World();
    robot = new Arm_Robot3(WORLD_ID, SPACE_ID);

    // First signal
    client.Create_connection();
    char SIGNAL[] = "RESTART_SIGNAL";
    client.Send(SIGNAL, sizeof(SIGNAL));
    client.Close_connection();
    
    
    // Simulation loop
    if(VIEW){
      // viewing mode
      dsSimulationLoop(argc, argv, 640, 570, &fn);
    }else{
      while(!ITER_FINISH){
	// without viewer
	simLoop(0);
      }
    }
    dCloseODE();
  }
  return 0;
}
