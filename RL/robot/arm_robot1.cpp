#include "include/arm_robot1.hpp"
#include <memory.h>

Arm_Robot1::Arm_Robot1(dWorldID world, dSpaceID space){
  // robot configuration
  dof = NUM - 1;
  angle_pitch = 5;
  double _ROM[DOF * 2] = {0, 140}; memcpy(ROM, _ROM, MEM_DOF*2);
  
  // Initialize variables
  double _ANGLE[DOF] = { 0.0 };
  double _l[NUM]  = { 0.10, 0.90 };
  double _r[NUM]  = { 0.1, 0.04 };
  dReal _tac_size[3] = {0.1, 0.1, 0.05};
  memcpy(ANGLE, _ANGLE, MEM_DOF);
  memcpy(l, _l, MEM_NUM);
  memcpy(r, _r, MEM_NUM);
  memcpy(tac_size, _tac_size, MEM_NUM);
  
  // center of position
  double _x[NUM] = {0.00};
  double _y[NUM] = {0.00};
  double _z[NUM] = { l[0]/2, l[0]+l[1]/2 };
  memcpy(x, _x, MEM_NUM);
  memcpy(y, _y, MEM_NUM);
  memcpy(z, _z, MEM_NUM);

  // mass
  double _m[NUM] = { 10.00, 2.00 };
  memcpy(m, _m, MEM_NUM);
  
  // tac sensor positon
  double tac_x[TAC_NUM] = { 0.00 };
  double tac_y[TAC_NUM] = { 0.00 };
  double tac_z[TAC_NUM] = { l[0]+l[1]+0.05 };

  // 回転中心
  double _anchor_x[DOF] = { 0.00 };
  double _anchor_y[DOF] = { 0.00 }; 
  double _anchor_z[DOF] = { l[0] };
  memcpy(anchor_x, _anchor_x, MEM_DOF);
  memcpy(anchor_y, _anchor_y, MEM_DOF);
  memcpy(anchor_z, _anchor_z, MEM_DOF);

  // 回転軸
  double _axis_x[DOF]  = { 0.00 }; 
  double _axis_y[DOF]  = { 1.00 }; 
  double _axis_z[DOF]  = { 0.00 }; 
  memcpy(axis_x, _axis_x, MEM_DOF);
  memcpy(axis_y, _axis_y, MEM_DOF);
  memcpy(axis_z, _axis_z, MEM_DOF);
  
  // Create manipulator
  for (int i=0; i<NUM; i++) {
    // body
    link[i].body = dBodyCreate(world);
    dBodySetPosition(link[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetCappedCylinderTotal(&mass,m[i],3,r[i],l[i]);
    dBodySetMass(link[i].body, &mass);
    
    // geom
    link[i].geom = dCreateCapsule(space, r[i], l[i]);
    dGeomSetBody(link[i].geom, link[i].body);
  }

  // Create a Tactile sensor
  for(int i=0; i<TAC_NUM; i++){
    // body
    tac_sensor[i].body = dBodyCreate(world);
    dBodySetPosition(tac_sensor[i].body, tac_x[i], tac_y[i], tac_z[i]);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, 1.0, tac_size[0], tac_size[1], tac_size[2]);
    dBodySetMass(tac_sensor[i].body, &mass);
    
    // geom
    tac_sensor[i].geom = dCreateBox(space, tac_size[0], tac_size[1], tac_size[2]);
    dGeomSetBody(tac_sensor[i].geom, tac_sensor[i].body);
  }
  
  // Setting Fix Joint
  body_fix[0] = dJointCreateFixed(world, 0);
  dJointAttach(body_fix[0], link[0].body, 0);
  dJointSetFixed(body_fix[0]);

  // Setting Hinge Joint
  for (int j=0; j<DOF; j++) {
    joint[j] = dJointCreateHinge(world, 0);
    dJointAttach(joint[j], link[j+1].body, link[j].body);
    dJointSetHingeAnchor(joint[j], anchor_x[j], anchor_y[j],anchor_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]);
  }
  
  for(int j=0; j<TAC_NUM; j++){
    tac_fix[j] = dJointCreateFixed(world, 0);
    dJointAttach(tac_fix[j], link[1].body, tac_sensor[0].body);
    dJointSetFixed(tac_fix[j]);
  }
}

Arm_Robot1::~Arm_Robot1(){
}

void Arm_Robot1::control() {
  /***  PID  ****/
  static int step = 0;
  static double z[3*DOF] = {0};
  // k1:比例ゲイン,  fMax：最大トルク[Nm]
  double k1 =  0.1, k2 = 0.09, k3 = 0.1,  fMax  = 100.0;
  printf("\r%6d:",step++);
  for (int j=0; j<DOF; j++) {
    double tmpAngle = dJointGetHingeAngle(joint[j]) / M_PI * 180;
    z[3 * j + 2] = z[3 * j + 1];
    z[3 * j + 1] = z[3 * j];
    z[3 * j] = ANGLE[j] - tmpAngle;
    double z_sum = 0;
    for(int i=0; i<3; i++) z_sum += z[3 * j + i];
    double omega = k1 * z[3 * j] + k2 * z_sum + k3 * (z[3 * j] - z[3* j + 1]);
    // 角速度の設定
    dJointSetHingeParam(joint[j],  dParamVel,  omega);
    // 最大トルクの設定
    dJointSetHingeParam(joint[j], dParamFMax, fMax);
  }
}

bool Arm_Robot1::Is_Tactile(dGeomID obj){
  for(int i=0; i<TAC_NUM; i++)
    if(tac_sensor[i].geom == obj) return true;
  else return false;
}

int Arm_Robot1::Which_Tactile(dGeomID obj){
  for(int i=0; i<TAC_NUM; i++)
    if(tac_sensor[i].geom == obj) return i;
  else return -1;
}

bool Arm_Robot1::BodyTactileCollision(dGeomID obj1, dGeomID obj2){
  for(int i=0; i<TAC_NUM; i++)
    for(int j=0; j<NUM; j++)
      if(tac_sensor[i].geom == obj1 && link[j].geom == obj2
	 ||
	 tac_sensor[i].geom == obj2 && link[j].geom == obj1)
	return true;
  return false;
}

void Arm_Robot1::command(int cmd){
}

Arm_Robot1* robot;
