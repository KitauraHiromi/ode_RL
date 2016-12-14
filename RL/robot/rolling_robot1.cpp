#include "include/rolling_robot1.hpp"
#include <memory.h>
#include <iostream>

Rolling_Robot1::Rolling_Robot1(dWorldID world, dSpaceID space){
  // robot configuration
  dof = DOF;
  angle_pitch = 5;
  double _ROM[DOF * 2] = { -180, 180 };
  memcpy(ROM, _ROM, MEM_DOF*2);
  
  // Initialize variables
  double _ANGLE[DOF] = { 30.0 };
  double _l[NUM]  = { 0.5, 0.5, 0.5, 0.5 };
  double _r[NUM]  = { 0.1, 0.1, 0.1, 0.1 };
  dReal _tac_size[3] = {0.1, 0.1, 0.05};
  memcpy(ANGLE, _ANGLE, MEM_DOF);
  memcpy(l, _l, MEM_NUM);
  memcpy(r, _r, MEM_NUM);
  memcpy(tac_size, _tac_size, 3*sizeof(double));
  
  // center of position
  double _x[NUM] = { 0.00, 0.00, 0.00, 0.00 };
  double _y[NUM] = { 0.00, 0.00, 0.25, 0.25 };
  double _z[NUM] = { l[0]/2, l[0]+0.5+l[1]/2, l[0]/2, l[0]+0.5+l[1]/2 }; 
  memcpy(x, _x, MEM_NUM);
  memcpy(y, _y, MEM_NUM);
  memcpy(z, _z, MEM_NUM);
  
  // mass
  double _m_link[NUM] = { 2.00, 2.00, 2.00, 2.00 };
  double _m_join[DOF] = { 2.00 };
  memcpy(m_link, _m_link, MEM_NUM);
  memcpy(m_join, _m_join, MEM_DOF);

  // tac sensor position
  double tac_x[TAC_NUM] = { 0.00 };
  double tac_y[TAC_NUM] = { 0.00 };
  double tac_z[TAC_NUM] = { z[1]+l[1]/2+0.1 };
  
  // 回転中心
  double _anchor_x[2*DOF] = { 0.00, 0.00 };
  double _anchor_y[2*DOF] = { 0.00, y[2] };
  double _anchor_z[2*DOF] = { (z[0]+z[1])/2, (z[0]+z[1])/2 };
  memcpy(anchor_x, _anchor_x, 2*MEM_DOF);
  memcpy(anchor_y, _anchor_y, 2*MEM_DOF);
  memcpy(anchor_z, _anchor_z, 2*MEM_DOF);

  // 回転軸
  double _axis_x[2*DOF]  = { 0.00, 0.00 };
  double _axis_y[2*DOF]  = { 1.00, 1.00 };
  double _axis_z[2*DOF]  = { 0.00, 0.00 };
  memcpy(axis_x, _axis_x, 2*MEM_DOF);
  memcpy(axis_y, _axis_y, 2*MEM_DOF);
  memcpy(axis_z, _axis_z, 2*MEM_DOF);

  // この辺関数化してもよいかも
  // Create link
  for (int i = 0; i <NUM; i++) {
    // body
    link[i].body = dBodyCreate(world);
    dBodySetPosition(link[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass_link[i]);
    dMassSetCappedCylinderTotal(&mass_link[i], m_link[i], 3, r[i], l[i]);
    dBodySetMass(link[i].body, &mass_link[i]);
    
    // geom
    link[i].geom = dCreateCapsule(space, r[i], l[i]);
    dGeomSetBody(link[i].geom, link[i].body);
  }

  // Create joint cylinder
  for (int i = 0; i <DOF; i++) {
    // get joint rotation matrix
    dMatrix3 R;
    dRFromZAxis(R, axis_x[2*i], axis_y[2*i], axis_z[2*i]);
    
    // body
    joint_cyli[i].body = dBodyCreate(world);
    dBodySetPosition(joint_cyli[i].body, (x[2*i+2]+x[2*i])/2, (y[2*i+2]+y[2*i])/2, (z[2*i+1]+z[2*i])/2);
    dBodySetRotation(joint_cyli[i].body, R);
    dMassSetZero(&mass_join[i]);
    dMassSetCylinderTotal(&mass_join[i], m_join[i], 3, (z[2*i+1]-z[2*i])/2, y[2*i+2]-y[2*i]);
    dBodySetMass(joint_cyli[i].body, &mass_join[i]);
 
    // geom
    joint_cyli[i].geom = dCreateCylinder(space, r[i], l[i]);
    dGeomSetBody(joint_cyli[i].geom, joint_cyli[i].body);
  }
  
  
  // Create a Tactile sensor
  for(int i=0; i<TAC_NUM; i++){
    // body
    tac_sensor[i].body = dBodyCreate(world);
    dBodySetPosition(tac_sensor[i].body, tac_x[i], tac_y[i], tac_z[i]);
    dMassSetZero(&mass_tac[i]);
    dMassSetBoxTotal(&mass_tac[i], 0.01, tac_size[0], tac_size[1], tac_size[2]);
    dBodySetMass(tac_sensor[i].body, &mass_tac[i]);
    
    // geom
    tac_sensor[i].geom = dCreateBox(space, tac_size[0], tac_size[1], tac_size[2]);
    dGeomSetBody(tac_sensor[i].geom, tac_sensor[i].body);
  }
  
  // Setting Joint
  for (int j=0; j<2*DOF; j++) {
    joint[j] = dJointCreateHinge(world, 0);
    dJointAttach(joint[j], joint_cyli[0].body, link[2*j+1].body);
    dJointSetHingeAnchor(joint[j], anchor_x[j], anchor_y[j],anchor_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]);
  }

  // body fix
  for(int j=0; j<2*DOF; j++){
    body_join_fix[j] = dJointCreateFixed(world, 0);
    dJointAttach(body_join_fix[j], link[2*j].body, joint_cyli[0].body);
    dJointSetFixed(body_join_fix[j]);
  }
  
  for(int j=0; j<NUM/2; j++){
    body_body_fix[j] = dJointCreateFixed(world, 0);
    dJointAttach(body_body_fix[j], link[j+NUM/2].body, link[j].body);
    dJointSetFixed(body_body_fix[j]);
  }
  
  // Fix tactile sensor to link
  for (int j=0; j<TAC_NUM; j++){
    tac_fix[j] = dJointCreateFixed(world, 0);
    dJointAttach(tac_fix[j], link[1].body, tac_sensor[j].body);
    dJointSetFixed(tac_fix[j]);
  }
}

Rolling_Robot1::~Rolling_Robot1(){
}

void Rolling_Robot1::control() {
  /***  PID  ****/
  static long int step = 0;
  static double z[3*DOF] = {0};
  // k1:比例ゲイン,  fMax：最大トルク[Nm]
  double k1 =  0.1, k2 = 0.09, k3 = 0.1,  fMax  = 100.0;
  printf("\r%6d:",step++);
  for (int j = 0; j <DOF; j++) {
    double tmpAngle = dJointGetHingeAngle(joint[j]) / M_PI * 180;
    z[3 * j + 2] = z[3 * j + 1];
    z[3 * j + 1] = z[3 * j];
    // z: 残差=目標関節角－現在関節角
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

bool Rolling_Robot1::Is_Tactile(dGeomID obj){
  for(int i=0; i<TAC_NUM; i++)
    if(tac_sensor[i].geom == obj) return true;
  else return false;
}

int Rolling_Robot1::Which_Tactile(dGeomID obj){
  for(int i=0; i<TAC_NUM; i++)
    if(tac_sensor[i].geom == obj) return i;
  else return -1;
}

bool Rolling_Robot1::BodyTactileCollision(dGeomID obj1, dGeomID obj2){
  for(int i=0; i<TAC_NUM; i++)
    for(int j=0; j<NUM; j++)
      if(tac_sensor[i].geom == obj1 && link[j].geom == obj2
	 ||
	 tac_sensor[i].geom == obj2 && link[j].geom == obj1)
	return true;
  return false;
}

Rolling_Robot1* robot;
