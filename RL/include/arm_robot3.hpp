#ifndef __ARM_ROBOT3_IS_USED__
#define __ARM_ROBOT3_IS_USED__

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#define NUM 4         // リンク数(土台含む)
#define DOF 3
#define MEM_NUM NUM*sizeof(double)
#define MEM_DOF DOF*sizeof(double)
#define TAC_NUM 1
#define Main_Robot Arm_Robot3

typedef struct {
  dBodyID body;
  dGeomID geom;
  dVector3 value;
} MyObject;

class Arm_Robot3{
 public:
  unsigned int dof;
  double angle_pitch;
  double ROM[DOF * 2];
  
  MyObject    link[NUM];         // リンク　link[0]は土台
  MyObject    tac_sensor[TAC_NUM]; 
  dJointID    body_fix[1];
  dJointID    joint[DOF];
  dJointID    tac_fix[TAC_NUM];
  double      ANGLE[DOF];        // 関節目標角[deg]
  double      l[NUM];            // リンク長[m]
  double      r[NUM];            // リンク半径[m]
  dReal       tac_size[3];

  // center of position
  double x[NUM];  double y[NUM];  double z[NUM];
  // mass
  double m[NUM];
 
  double tac_x[TAC_NUM];
  double tac_y[TAC_NUM];
  double tac_z[TAC_NUM];
  double anchor_x[DOF];  double anchor_y[DOF];  double anchor_z[DOF];// 回転中心
  double axis_x[DOF];  double axis_y[DOF];  double axis_z[DOF];      // 回転軸

  // mass parameter
  dMass mass;
  
  Arm_Robot3(dWorldID, dSpaceID);
  ~Arm_Robot3();
  void control();
  bool Is_Tactile(dGeomID);
  int Which_Tactile(dGeomID);
  bool BodyTactileCollision(dGeomID, dGeomID);
  void command(int);
};

extern Arm_Robot3* robot;

#endif
