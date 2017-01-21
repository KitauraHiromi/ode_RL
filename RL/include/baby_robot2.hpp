#ifndef __BABY_ROBOT2__
#define __BABY_ROBOT2__

#include <vector>
#include <string>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "include/libmesh.hpp"

// the number of links
#define NUM 11
#define DOF 15
#define SHELL_NUM 2
#define MEM_NUM NUM*sizeof(dReal)
#define MEM_DOF DOF*sizeof(dReal)
#define TAC_NUM 1
#define Main_Robot Baby_Robot2
//#define NO_SHELL

typedef struct {
  dBodyID body;
  dGeomID geom;
  dVector3 value;
} MyObject;

class Baby_Robot2{
public:
  //dReal angle_pitch;
  dReal ROM[DOF * 2];
  //unsigned int dof;

  MyObject    link[NUM];         // リンク
  MyObject    joint_cyli[DOF];
  MyObject    tac_sensor[TAC_NUM];
  MyObject    outer_shell[SHELL_NUM];
  dJointID    joint[DOF];      // 関節
  dJointID    body_join_fix[6];
  dJointID    limb_join_fix[4];
  dJointID    outer_shell_fix[SHELL_NUM];
  dJointID    tac_fix[TAC_NUM];// 触覚固定ジョイント
  dReal      ANGLE[DOF];      // 関節目標角[deg]
  dReal      l[NUM];          // リンク長[m]
  dReal      r[NUM];          // リンク半径[m]
  dReal      joint_cyli_r;
  dReal      tac_size[3];

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
  dMass mass_tac[TAC_NUM];
  
  Baby_Robot2(dWorldID, dSpaceID);
  ~Baby_Robot2();
  void Create_tac(dWorldID, dSpaceID);
  void Create_link(dWorldID, dSpaceID);
  void Create_joint_cyli(dWorldID, dSpaceID);
  void Joint_Setting(dWorldID);
  void restrict_angle(int);
  void control();
  bool Is_Tactile(dGeomID);
  int Which_Tactile(dGeomID);
  bool BodyTactileCollision(dGeomID, dGeomID);
  void command(int);
};

extern Baby_Robot2* robot;

#endif
