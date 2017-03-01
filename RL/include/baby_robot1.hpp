#ifndef __BABY_ROBOT1_IS_USED__
#define __BABY_ROBOT1_IS_USED__

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <cmath>
#include <include/tactile_sensor.hpp>

#define NUM 7
#define DOF 11
#define MEM_NUM NUM*sizeof(dReal)
#define MEM_DOF DOF*sizeof(dReal)
#define Main_Robot Baby_Robot1

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
  // 触覚はrayで実装
  //dJointID    tac_fix[TAC_NUM];// 触覚固定ジョイント
  dReal      ANGLE[DOF];      // 関節目標角[deg]
  dReal      l[NUM];          // リンク長[m]
  dReal      r[NUM];          // リンク半径[m]
  
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

  // tactile sensor sheet
  Tac_Sheet* Right_Arm;
  Tac_Sheet* Left_Arm;
  Tac_Sheet* Right_Leg;
  Tac_Sheet* Left_Leg;
  Tac_Sheet* Upper_Trunk;
  Tac_Sheet* Lower_Trunk;
  
  Baby_Robot1(dWorldID, dSpaceID);
  ~Baby_Robot1();
  void Write_Pos(std::ofstream&, int);
  void Write_Act(std::ofstream&, int);
  void Restrict_Angle(int);
  void Control();
  bool Is_Tactile(dGeomID);
  int Which_Tactile(dGeomID);
  bool Body_Tactile_Collision(dGeomID, dGeomID);
  void Command(int);
};

extern Baby_Robot1* robot;

#endif
