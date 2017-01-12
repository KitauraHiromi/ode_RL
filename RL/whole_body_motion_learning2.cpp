#include "include/whole_body_motion_learning2.hpp"
#define ONE_ITER_LENGTH 10000
#define SIGNAL_SPAN 10
#define DEBUG 1

bool __VIEW__ = false;
bool ITER_FINISH = false;
int count, sum_count;

// TCP/IP communication
Client client = Client();

void restart(Main_Robot* robot){
  client.Create_connection();
  char SIGNAL[] = "RESTART_SIGNAL";
  client.Send(SIGNAL, sizeof(SIGNAL));
  client.Close_connection();

  // simulation reset. maybe including some bugs.
  delete robot;
  robot = new Main_Robot(WORLD_ID, SPACE_ID);
  printf("\nRESTART\n");
  count = 0;
}

void command(int cmd){
  robot->command(cmd);
}

void start(){
  // Initialize drawing API
  float xyz[3] = {  3.04, 1.28, 0.76 };
  float hpr[3] = {-160.0, 4.50, 0.00 };
  dsSetViewpoint(xyz, hpr);
}

void simLoop(int pause) {
  // Call dSpaceCollide in the begining of simLoop.
  // dSpaceCollide calls nearCallback.
  dSpaceCollide(SPACE_ID, 0, &nearCallback); 
  robot->control();
  // DT is written in nearCallback.hpp
  dWorldStep(WORLD_ID, DT);
  //dWorldQuickStep ( WORLD_ID, DT );
  dJointGroupEmpty(world.contactgroup);


#if !DEBUG
  // TCP/IP communication
  // communication protcol should be written in config file. 
  if(count % SIGNAL_SPAN == 1){
    // description
    std::stringstream ss;
    char recieve_buf[256];
    ss << count;
    for(int i=0; i<DOF; i++){
      robot->ANGLE[i] = dJointGetHingeAngle(robot->joint[i]) * 180 / M_PI;
      ss << ',' << robot->ANGLE[i];
    }
    for(int i=0; i<TAC_NUM; i++) ss << ',' << robot->tac_sensor[i].value[2];
    char* send_buf = new char[ss.str().length()+1];
    strcpy(send_buf, ss.str().c_str());
    
    bool TCP_SUCCEED = false;
    int n = -1;
    while(!TCP_SUCCEED){
      client.Create_connection();
      client.Send(send_buf, strlen(send_buf));
      n = client.Recieve(recieve_buf, sizeof(recieve_buf));
      if(n != -1) TCP_SUCCEED = true;
      client.Close_connection();
    }
    
    // Robot move
    const std::string str(recieve_buf);
    std::istringstream iss(str);
    for(int i=0; i<robot->dof; i++)
      iss >> robot->ANGLE[i];
  }
#endif
  
  
  // Drawing robot
  if(__VIEW__){
    dsSetColor(1.0,1.0,1.0);
    
    for (int i = 0; i <NUM; i++ ){
      dsDrawCapsuleD(dBodyGetPosition(robot->link[i].body),
		     dBodyGetRotation(robot->link[i].body),
		     robot->l[i],
		     robot->r[i]);
    }
    
    for(int i=0; i<DOF; i++){
      dsDrawCylinderD(dBodyGetPosition(robot->joint_cyli[i].body),
		      dBodyGetRotation(robot->joint_cyli[i].body),
		      robot->joint_cyli_r,
		      robot->joint_cyli_r);
    }
    
    dsSetColor(1.0, 0.0, 0.0);
    
    
    for(int i=0; i<TAC_NUM; i++)
      dsDrawBoxD(dBodyGetPosition(robot->tac_sensor[i].body),
		 dBodyGetRotation(robot->tac_sensor[i].body),
		 robot->tac_size);
    
  }
  drawMesh(robot->outer_shell1.geom);
  
  // counter increment
  ++sum_count;
  if(++count > ONE_ITER_LENGTH){
    ITER_FINISH = true;
    count = 0;
  }
}

int main(int argc, char *argv[]) {
  dsFunctions fn; // drawing function
  bool FIRST = true;

  while(1){
  // Initialization
  dInitODE();
  ITER_FINISH = false;
  world = World();
  robot = new Main_Robot(WORLD_ID, SPACE_ID);
  // getting initial angles and tactiles

#if !DEBUG
  // Start of iteration signal
  if(FIRST){
#endif
  
  if(argc > 1){
  __VIEW__ = true;
  printf("%s\n", argv[1]);
}
  
  if(__VIEW__){
  fn.version = DS_VERSION;     fn.start = &start;
  fn.step   = &simLoop;        fn.command = &command;
  fn.path_to_textures = TEXTURE_PATH;
}
  
#if !DEBUG
  client.Create_connection();
  // "SIGNAL,DOF,angle_pitch,ROM[i],..."
  std::stringstream ss;
  ss << "FIRST_ITER_SIGNAL,"
     << robot->dof << ','
     << TAC_NUM << ','
     << robot->angle_pitch;
  for(int i=0; i<DOF*2; i++) ss << ',' << robot->ROM[i];
  
  char* SIGNAL = new char[ss.str().length()+1];
  strcpy(SIGNAL, ss.str().c_str());
  std::cout << SIGNAL << " " << strlen(SIGNAL)<< std::endl;
  client.Send(SIGNAL, strlen(SIGNAL));
  client.Close_connection();
  FIRST = false;
}else{
  client.Create_connection();
  char SIGNAL[] = "NEXT_ITER_SIGNAL";
  client.Send(SIGNAL, strlen(SIGNAL));
  client.Close_connection();
}
#endif
  
  // Simulation loop
  if(__VIEW__){
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
