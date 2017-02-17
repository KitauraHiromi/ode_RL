#include <include/whole_body_motion_learning1.hpp>
#define ONE_ITER_LENGTH 10000
#define FREQ 100
#define SIGNAL_SPAN 10
#define DEBUG 1

bool __VIEW__ = false;
bool ITER_FINISH = false;
int count, sum_count;  

std::ofstream tac_out("tac.log");
std::ofstream pos_out("pos.log");

// TCP/IP communication
Client client = Client();

void restart(Main_Robot* robot){
  delete robot;
  robot = new Main_Robot(WORLD_ID, SPACE_ID);
  printf("\nRESTART\n");
  count = 0;
}

void command(int cmd){
  robot->Command(cmd);
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
  robot->Control();
  // DT is written in nearCallback.hpp
  dWorldStep(WORLD_ID, DT);
  //dWorldQuickStep ( WORLD_ID, DT );
  dJointGroupEmpty(world.contactgroup);

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
		      0.05,
		      0.05);
    }
    
    dsSetColor(1.0, 0.0, 0.0);
    
#ifdef __USE_TACTILE__
    robot->Right_Arm->Draw_Sheet();
    robot->Left_Arm->Draw_Sheet();
    robot->Right_Leg->Draw_Sheet();
    robot->Left_Leg->Draw_Sheet();
    robot->Upper_Trunk->Draw_Sheet();
    robot->Lower_Trunk->Draw_Sheet();
   
    if(count % FREQ == 0){
      robot->Right_Arm->Set_Tactile_Values();   robot->Right_Arm->Write_Data(tac_out);
      robot->Left_Arm->Set_Tactile_Values();    robot->Left_Arm->Write_Data(tac_out);
      robot->Right_Leg->Set_Tactile_Values();   robot->Right_Leg->Write_Data(tac_out);
      robot->Left_Leg->Set_Tactile_Values();    robot->Left_Leg->Write_Data(tac_out);
      robot->Upper_Trunk->Set_Tactile_Values(); robot->Upper_Trunk->Write_Data(tac_out);
      robot->Lower_Trunk->Set_Tactile_Values(); robot->Lower_Trunk->Write_Data(tac_out);
    }
    // todo 触覚の可視化

    
    // for (int i=0; i<robot->Upper_Trunk->axis_num; i++){
    //   for (int j=0; j<robot->Upper_Trunk->round_num; j++){
    // 	dReal val = robot->Upper_Trunk->tac_sensors[i*robot->Upper_Trunk->round_num + j].value[1];
    // 	//if(val < 0.0001) val = 0;
    // 	std::cout << val << " ";
    //   }
    //   std::cout << std::endl; 
    //}
    
#endif
  }
  
    // counter increment
    ++sum_count;
    if(++count > ONE_ITER_LENGTH){
      std::cout << count << std::endl;
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
    
    // Start of iteration signal
    if(FIRST){
      
      
      if(argc > 1){
	__VIEW__ = true;
	printf("%s\n", argv[1]);
      }
      
      if(__VIEW__){
	fn.version = DS_VERSION;     fn.start = &start;
	fn.step   = &simLoop;        fn.command = &command;
	fn.path_to_textures = TEXTURE_PATH;
      }
      
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
  }
  return 0;
}
