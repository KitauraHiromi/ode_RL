#ifndef __BABY_ROBOT1__
#define __BABY_ROBOT1__

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#define NUM 7
#define DOF 11
#define MEM_NUM NUM*sizeof(dReal)
#define MEM_DOF DOF*sizeof(dReal)
#define TAC_NUM 0
#define Main_Robot Baby_Robot1

typedef struct {
  dBodyID body;
  dGeomID geom;
  dVector3 value;
} MyObject;

class Baby_Robot1{
public:
  dReal angle_pitch;
  dReal ROM[DOF * 2];
  unsigned int dof;

  MyObject    link[NUM];         // リンク
  MyObject    joint_cyli[DOF];
  //MyObject    tac_sensor[TAC_NUM];
  dJointID    joint[DOF];      // 関節
  dJointID    body_join_fix[6];
  //dJointID    tac_fix[TAC_NUM];// 触覚固定ジョイント
  dReal      ANGLE[DOF];      // 関節目標角[deg]
  dReal      l[NUM];          // リンク長[m]
  dReal      r[NUM];          // リンク半径[m]
  dReal       tac_size[3];

  // center of positon
  dReal x[NUM];  dReal y[NUM];  dReal z[NUM];
  //mass
  dReal m_link[NUM];
  dReal m_join[DOF];
  
  dReal x1, y1, z1;
  dReal anchor_x[DOF];  dReal anchor_y[DOF];  dReal anchor_z[DOF];// 回転中心
  dReal axis_x[DOF];  dReal axis_y[DOF];  dReal axis_z[DOF];      // 回転軸

  // mass parameter
  dMass mass_link[NUM];
  dMass mass_join[DOF];
  //dMass mass_tac[TAC_NUM];
  
  Baby_Robot1(dWorldID, dSpaceID);
  ~Baby_Robot1();
  void restrict_angle(int);
  void control();
  //bool Is_Tactile(dGeomID);
  //int Which_Tactile(dGeomID);
  //bool BodyTactileCollision(dGeomID, dGeomID);
  void command(int);
};

extern Baby_Robot1* robot;

#endif
