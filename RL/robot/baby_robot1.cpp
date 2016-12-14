#include <include/baby_robot1.hpp>
#include <memory.h>
#include <iostream>

Baby_Robot1::Baby_Robot1(dWorldID world, dSpaceID space){
  // robot configuration
  dof = DOF;
  angle_pitch = 60;
  dReal _ROM[DOF*2] = { -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90 };
  memcpy(ROM, _ROM, MEM_DOF*2);

  // Rotation Matrix
  dReal angle_y = 90;
  dReal angle_x = 90;
  dMatrix3 R_y, R_x, R_init;
  dRFromAxisAndAngle(R_y, 0, 1, 0, M_PI*angle_y/180.0);
  dRFromAxisAndAngle(R_x, 1, 0, 0, M_PI*angle_x/180.0);
  dMultiply0(R_init, R_y, R_x, 3, 3, 3);
  
  // Initialize variables
  dReal _ANGLE[DOF] = { 0.0 };
  // left leg, right leg, left arm, right arm, upper trunk, lower trunk, head
  //                  ll   , rl   , la   , ra   , ut   , lt   , h
  dReal _l[NUM]  = { 0.300, 0.300, 0.250, 0.250, 0.100, 0.100, 0.1 };
  //dReal _l[NUM]  = { 0.600, 0.600, 0.5, 0.5, 0.200, 0.200, 0.200 };
  dReal _r[NUM]  = { 0.075, 0.075, 0.050, 0.050, 0.175, 0.150, 0.15 };
  dReal _tac_size[3] = {0.1, 0.1, 0.05};
  memcpy(ANGLE, _ANGLE, MEM_DOF);
  memcpy(l, _l, MEM_NUM);
  memcpy(r, _r, MEM_NUM);
  memcpy(tac_size, _tac_size, 3*sizeof(dReal));
  
  dReal dist = 0.1;
  // center of position
  //                                ll,                rl,             la,         ra,           ut,      lt,                  h 
  dReal _x[NUM] = {    -1*(r[0]+r[5]),         r[1]+r[5], -1*(r[2]+r[4]),  r[2]+r[4],        0.000,   0.000,              0.000 };
  dReal _y[NUM] = { -l[0]/2-l[5]-dist, -l[0]/2-l[5]-dist,     0.050+dist, 0.050+dist, 0.000+dist*3, -l[5]/2, l[4]+l[6]/2+dist*5 };
  dReal _z[NUM] = {            l[4]/2,            l[4]/2,         l[4]/2,     l[4]/2,       l[4]/2,  l[4]/2,        l[4]/2-0.03 }; 
  memcpy(x, _x, MEM_NUM);
  memcpy(y, _y, MEM_NUM);
  memcpy(z, _z, MEM_NUM);
  
  // mass
  dReal _m_link[NUM] = { 1.00, 1.00, 0.75, 0.75, 4.00, 3.00, 3.00 };
  dReal _m_join[DOF] = { 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 };
  memcpy(m_link, _m_link, MEM_NUM);
  memcpy(m_join, _m_join, MEM_DOF);

  // tac sensor position
  //dReal tac_x[TAC_NUM] = { 0.00 };
  //dReal tac_y[TAC_NUM] = { 0.00 };
  //dReal tac_z[TAC_NUM] = { z[1]+l[1]/2+0.1 };

  dReal d = 0.1;
  // 回転中心                        ll-z,          ll-x,          rl-z,          rl-x,          la-z,          la-y,          ra-z,          ra-y,         t-y,         t-x,         h-y
  dReal _anchor_x[DOF] = {          x[0],          x[0],          x[1],          x[1],          x[2],          x[2],          x[3],          x[3],        x[4],        x[4],        x[6]};
  dReal _anchor_y[DOF] = { y[0]+l[0]/2+d, y[0]+l[0]/2+d, y[1]+l[1]/2+d, y[1]+l[1]/2+d, y[2]+l[2]/2+d, y[2]+l[2]/2+d, y[3]+l[3]/2+d, y[3]+l[3]/2+d, y[4]-l[4]/2, y[4]-l[4]/2, y[6]-l[6]/2};
  dReal _anchor_z[DOF] = {          z[0],          z[0],          z[1],          z[1],          z[2],          z[2],          z[3],          z[3],        z[4],        z[4],        z[6]};
  memcpy(anchor_x, _anchor_x, MEM_DOF);
  memcpy(anchor_y, _anchor_y, MEM_DOF);
  memcpy(anchor_z, _anchor_z, MEM_DOF);

  // 回転軸                ll-z,  ll-x,  rl-z, rl-x, la-z,  la-y,  ra-z, ra-y,  t-y,  t-x,  h-y
  dReal _axis_x[DOF]  = { 0.00, -1.00,  0.00, 1.00, 0.00,  0.00,  0.00, 0.00, 0.00, 1.00, 0.00 };
  dReal _axis_y[DOF]  = { 0.00,  0.00,  0.00, 0.00, 0.00, -1.00,  0.00, 1.00, 1.00, 0.00, 1.00 };
  dReal _axis_z[DOF]  = { 1.00,  0.00, -1.00, 0.00, 1.00,  0.00, -1.00, 0.00, 0.00, 0.00, 0.00 };
  memcpy(axis_x, _axis_x, MEM_DOF);
  memcpy(axis_y, _axis_y, MEM_DOF);
  memcpy(axis_z, _axis_z, MEM_DOF);
  
  // この辺関数化してもよいかも
  // Create link
  
  for (int i = 0; i <NUM; i++) {
    // body
    link[i].body = dBodyCreate(world);
    //dBodySetGyroscopicMode(link[i].body, 0);
    dBodySetPosition(link[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass_link[i]);
    dMassSetCappedCylinderTotal(&mass_link[i], m_link[i], 3, r[i], l[i]);
    dBodySetMass(link[i].body, &mass_link[i]);
    dBodySetRotation(link[i].body, R_init);
    
    // geom
    link[i].geom = dCreateCapsule(space, r[i], l[i]);
    dGeomSetBody(link[i].geom, link[i].body);
  }
  
  
  // Create joint cylinder
  
  for (int i = 0; i <DOF; i++) {
    // get joint rotation matrix
    dMatrix3 R;
    dRFromZAxis(R, axis_x[i], axis_y[i], axis_z[i]);
    
    // body
    // direction may be wrong.
    joint_cyli[i].body = dBodyCreate(world);
    //dBodySetGyroscopicMode(joint_cyli[i].body, 0);
    dBodySetPosition(joint_cyli[i].body, anchor_x[i], anchor_y[i], anchor_z[i]);
    dBodySetRotation(joint_cyli[i].body, R);
    dMassSetZero(&mass_join[i]);
    dMassSetCylinderTotal(&mass_join[i], m_join[i], 1, 0.05, 0.05);
    dBodySetMass(joint_cyli[i].body, &mass_join[i]);
 
    // geom
    joint_cyli[i].geom = dCreateCylinder(space, 0.05, 0.05);
    dGeomSetBody(joint_cyli[i].geom, joint_cyli[i].body);
  }
  
 
  // Setting Joint
  for (int j=0; j<DOF; j++) {
    joint[j] = dJointCreateHinge(world, 0);
  }
  for(int j=0; j<6; j++){
    body_join_fix[j] = dJointCreateFixed(world, 0);
  }
  
  // left leg to lower trunk
  dJointAttach(        joint[0],       link[0].body,  joint_cyli[0].body);
  dJointAttach(        joint[1], joint_cyli[0].body,  joint_cyli[1].body);
  dJointAttach(body_join_fix[0], joint_cyli[1].body,        link[5].body);
  // right leg to lower trunk
  dJointAttach(        joint[2],       link[1].body,  joint_cyli[2].body);
  dJointAttach(        joint[3], joint_cyli[2].body,  joint_cyli[3].body);
  dJointAttach(body_join_fix[1], joint_cyli[3].body,        link[5].body);
  // left arm to upper trunk
  dJointAttach(        joint[4],       link[2].body,  joint_cyli[4].body);
  dJointAttach(        joint[5], joint_cyli[4].body,  joint_cyli[5].body);
  dJointAttach(body_join_fix[2], joint_cyli[5].body,        link[4].body);
  // right arm to upper trunk
  dJointAttach(        joint[6],       link[3].body,  joint_cyli[6].body);
  dJointAttach(        joint[7], joint_cyli[6].body,  joint_cyli[7].body);
  dJointAttach(body_join_fix[3], joint_cyli[7].body,        link[4].body);
  // upper trunk to lower trunk
  dJointAttach(        joint[8],       link[4].body,  joint_cyli[8].body);
  dJointAttach(        joint[9], joint_cyli[8].body,  joint_cyli[9].body);
  dJointAttach(body_join_fix[4], joint_cyli[9].body,        link[5].body);
  // head to upper trunk
  dJointAttach(       joint[10],        link[4].body, joint_cyli[10].body);
  dJointAttach(body_join_fix[5], joint_cyli[10].body,        link[6].body);

  for(int j=0; j<DOF; j++){
    dJointSetHingeAnchor(joint[j], anchor_x[j], anchor_y[j],anchor_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]);
  }
  for(int j=0; j<6; j++)
    dJointSetFixed(body_join_fix[j]);

  // Create a Tactile sensor
  /*
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
  */
  
  /*
  // Fix tactile sensor to link
  for (int j=0; j<TAC_NUM; j++){
    tac_fix[j] = dJointCreateFixed(world, 0);
    dJointAttach(tac_fix[j], link[1].body, tac_sensor[j].body);
    dJointSetFixed(tac_fix[j]);
  }
  */
}

Baby_Robot1::~Baby_Robot1(){
}

void Baby_Robot1::restrict_angle(int i){
  ANGLE[i] = std::max(ROM[2*i], ANGLE[i]);
  ANGLE[i] = std::min(ROM[2*i+1], ANGLE[i]);
}


void Baby_Robot1::control() {
  /***  PID  ****/
  static long int step = 0;
  static dReal z[3*DOF] = {0};
  // k1:比例ゲイン,  fMax：最大トルク[Nm]
  dReal k1 =  0.1, k2 = 0.09, k3 = 0.1,  fMax  = 10.0;
  printf("\r%6d:",step++);
  for (int j = 0; j <DOF; j++) {
    dReal tmpAngle = dJointGetHingeAngle(joint[j]) / M_PI * 180;
    z[3 * j + 2] = z[3 * j + 1];
    z[3 * j + 1] = z[3 * j];
    // z: 残差=目標関節角－現在関節角
    z[3 * j] = ANGLE[j] - tmpAngle;
    dReal z_sum = 0;
    for(int i=0; i<3; i++) z_sum += z[3 * j + i];
    dReal omega = k1 * z[3 * j] + k2 * z_sum + k3 * (z[3 * j] - z[3* j + 1]);
    // 角速度の設定
    dJointSetHingeParam(joint[j],  dParamVel,  omega);
    // 最大トルクの設定
    dJointSetHingeParam(joint[j], dParamFMax, fMax);
  }
}
/*
bool Baby_Robot1::Is_Tactile(dGeomID obj){
  for(int i=0; i<TAC_NUM; i++)
    if(tac_sensor[i].geom == obj) return true;
  else return false;
}

int Baby_Robot1::Which_Tactile(dGeomID obj){
  for(int i=0; i<TAC_NUM; i++)
    if(tac_sensor[i].geom == obj) return i;
  else return -1;
}

bool Baby_Robot1::BodyTactileCollision(dGeomID obj1, dGeomID obj2){
  for(int i=0; i<TAC_NUM; i++)
    for(int j=0; j<NUM; j++)
      if(tac_sensor[i].geom == obj1 && link[j].geom == obj2
	 ||
	 tac_sensor[i].geom == obj2 && link[j].geom == obj1)
	return true;
  return false;
}
*/

void Baby_Robot1::command(int cmd) {
  switch (cmd) {
    // case 'r': restart(robot); break;
  case '1': ANGLE[0] += 5; restrict_angle(0); break;
  case '2': ANGLE[1] += 5; restrict_angle(1); break;
  case '3': ANGLE[2] += 5; restrict_angle(2); break;
  case '4': ANGLE[3] += 5; restrict_angle(3); break;
  case '5': ANGLE[4] += 5; restrict_angle(4); break;
  case '6': ANGLE[5] += 5; restrict_angle(5); break;
  case '7': ANGLE[6] += 5; restrict_angle(6); break;
  case '8': ANGLE[7] += 5; restrict_angle(7); break;
  case '9': ANGLE[8] += 5; restrict_angle(8); break; 
  case '0': ANGLE[9] += 5; restrict_angle(9); break; 
  case '-': ANGLE[10] += 5; restrict_angle(10); break;
    
  case 'q': ANGLE[0] -= 5; restrict_angle(0); break;
  case 'w': ANGLE[1] -= 5; restrict_angle(1); break;
  case 'e': ANGLE[2] -= 5; restrict_angle(2); break;
  case 'r': ANGLE[3] -= 5; restrict_angle(3); break;
  case 't': ANGLE[4] -= 5; restrict_angle(4); break;
  case 'y': ANGLE[5] -= 5; restrict_angle(5); break;
  case 'u': ANGLE[6] -= 5; restrict_angle(6); break;
  case 'i': ANGLE[7] -= 5; restrict_angle(7); break;
  case 'o': ANGLE[8] -= 5; restrict_angle(8); break; 
  case 'p': ANGLE[9] -= 5; restrict_angle(9); break; 
  case '@': ANGLE[10] -= 5; restrict_angle(10); break; 
  }
}


Baby_Robot1* robot;
