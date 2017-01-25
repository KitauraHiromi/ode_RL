#ifndef __ROLLING_ROBOT1_IS_USED__
#define __ROLLING_ROBOT1_IS_USED__

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#define NUM 2*2
#define DOF 1
#define MEM_NUM NUM*sizeof(double)
#define MEM_DOF DOF*sizeof(double)
#define TAC_NUM 1
#define Main_Robot Rolling_Robot1

typedef struct {
  dBodyID body;
  dGeomID geom;
  dVector3 value;
} MyObject;

class Rolling_Robot1{
public:
  double angle_pitch;
  double ROM[DOF * 2];
  unsigned int dof;

  MyObject    link[NUM];         // リンク
  MyObject    joint_cyli[DOF];
  MyObject    tac_sensor[TAC_NUM];
  dJointID    body_join_fix[2*DOF];
  dJointID    body_body_fix[NUM/2];
  dJointID    joint[2*DOF];      // 関節
  dJointID    tac_fix[TAC_NUM];// 触覚固定ジョイント
  double      ANGLE[DOF];      // 関節目標角[deg]
  double      l[NUM];          // リンク長[m]
  double      r[NUM];          // リンク半径[m]
  dReal       tac_size[3];

  // center of positon
  double x[NUM];  double y[NUM];  double z[NUM];
  //mass
  double m_link[NUM];
  double m_join[DOF];
  double tac_x[TAC_NUM]; double tac_y[TAC_NUM]; double tac_z[TAC_NUM];
  double anchor_x[2*DOF];  double anchor_y[2*DOF];  double anchor_z[2*DOF];// 回転中心
  double axis_x[2*DOF];  double axis_y[2*DOF];  double axis_z[2*DOF];      // 回転軸

  // mass parameter
  dMass mass_link[NUM];
  dMass mass_join[DOF];
  dMass mass_tac[TAC_NUM];
  
  Rolling_Robot1(dWorldID, dSpaceID);
  ~Rolling_Robot1();
  void control();
  bool Is_Tactile(dGeomID);
  int Which_Tactile(dGeomID);
  bool BodyTactileCollision(dGeomID, dGeomID);
};

extern Rolling_Robot1* robot;

#endif
