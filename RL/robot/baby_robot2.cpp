#include <include/baby_robot2.hpp>
#include <memory.h>
#include <iostream>

Baby_Robot2::Baby_Robot2(dWorldID world, dSpaceID space){
  // robot configuration
  dReal _ROM[DOF*2] = { -90, 10,
			-90, 90,
			-90, 90,
			-90, 10,
			-90, 90,
			-90, 90,
			-90, 90,
			-90, 90,
			-90, 90,
			-90, 90,
			-90, 90,
			-90, 90,
			-90, 90,
			-90, 90,
			-90, 90  };
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
  // left uppper leg, left lower leg, right upper leg, right lower leg, left upper arm, left lower arm, right upper arm, right lower arm, upper trunk, lower trunk, head
  dReal _l[NUM]  = { 0.105, // lul
		     0.150, // lll
		     0.105, // rul
		     0.150, // rll
		     0.065, // lua
		     0.080, // lla
		     0.065, // rua
		     0.080, // rla
		     0.100, // ut
		     0.100, // lt
		     0.075   }; // h
  
  dReal _r[NUM]  = { 0.020, // lul
		     0.020, // lll
		     0.020, // rul
		     0.020, // rll
		     0.020, // lua
		     0.020, // lla
		     0.020, // rua
		     0.020, // rla
		     0.020, // ut
		     0.020, // lt
		     0.020  }; // h
  dReal _tac_size[3] = {0.005, 0.005, 0.0025};
  memcpy(ANGLE, _ANGLE, MEM_DOF);
  memcpy(l, _l, MEM_NUM);
  memcpy(r, _r, MEM_NUM);
  memcpy(tac_size, _tac_size, 3*sizeof(dReal));
  
  dReal dist = 0.1;
  dReal d2 = 0.02;
  
  // mass
  dReal _m_link[NUM] = { 0.500, 0.500, 0.500, 0.500, 0.375, 0.375, 0.375, 0.375, 4.000, 3.000, 3.000 };
  dReal _m_join[DOF] = { 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010, 0.010 };
  memcpy(m_link, _m_link, MEM_NUM);
  memcpy(m_join, _m_join, MEM_DOF);


  dReal d = 0.1;
  joint_cyli_r = 0.02;
  
  // 回転中心
  
  dReal _anchor_x[DOF] = { -0.06, // lul-z
			   -0.06, // lul-x
			   -0.06, // lll-x
			   0.06, // rul-z
			   0.06, // rul-x
			   0.06, // rll-x
			   -0.110, // lua-z
			   -0.110, // lua-y
			   -0.211, // lla-x
			   0.110, // rua-z
			   0.110, // rua-y
			   0.211, // rla-x
			   0.000, // t-y
			   0.000, // t-x
			   0.000};// h-y
 
  dReal _anchor_y[DOF] = { -0.090 - 0.113, // lul-z
			   -0.090 - 0.113, // lul-x
			   -0.211 - 0.113, // lll-x
			   -0.090 - 0.113, // rul-z
			   -0.090 - 0.113, // rul-x
			   -0.211 - 0.113, // rll-x
			   0.000, // lua-z
			   0.000, // lua-y
			   0.000, // lla-x
			   0.000, // rua-z
			   0.000, // rua-y
			   0.000, // rla-x
			   -0.113,   // t-y
			   -0.113,   // t-x
			   0.090};  // h-y
  
  dReal _anchor_z[DOF] = { 0.050, // lul-z
			   0.050, // lul-x
			   0.050, // lll-x
			   0.050, // rul-z
			   0.050, // rul-x
			   0.050, // rll-x
			   0.050, // lua-z
			   0.050, // lua-y
			   0.050, // lla-y
			   0.050, // rua-z
			   0.050, // rua-y
			   0.050, // rla-y
			   0.050, // t-y
			   0.050, // t-x
			   0.050};// h-y
  
  memcpy(anchor_x, _anchor_x, MEM_DOF);
  memcpy(anchor_y, _anchor_y, MEM_DOF);
  memcpy(anchor_z, _anchor_z, MEM_DOF);

  // center of link position
  dReal _x[NUM] = {(anchor_x[0] + anchor_x[2])/2., // lul
		   anchor_x[2], // lll
		   (anchor_x[3] + anchor_x[5])/2., // rul
		   anchor_x[5], // rll
		   (anchor_x[6] + anchor_x[8])/2., // lua
		   anchor_x[8] - (l[5]/2. + 2*joint_cyli_r),// lla
		   (anchor_x[9] + anchor_x[11])/2., // rua
		   anchor_x[11] + l[7]/2. + 2*joint_cyli_r, // rla
		   0.000, // ut
		   0.000, // lt
		   0.000 }; // h
  
  dReal _y[NUM] = {(anchor_y[0] + anchor_y[2])/2., // lul
		   anchor_y[2] - (l[1]/2. + 2*joint_cyli_r), // lll
		   (anchor_y[3] + anchor_y[5])/2., // rul
		   anchor_y[5] - (l[3]/2. + 2*joint_cyli_r), // rll
		   (anchor_y[6] + anchor_y[8])/2., // lua
		   (anchor_y[6] + anchor_y[8])/2., // lla
		   (anchor_y[9] + anchor_y[11])/2., // rua
		   (anchor_y[9] + anchor_y[11])/2., // rla
		   anchor_y[12] + l[8]/2. + 2*joint_cyli_r, // ut
		   anchor_y[13] - l[9]/2. - 2*joint_cyli_r, // lt
		   anchor_y[14] + l[10]/2. + 2*joint_cyli_r}; // h
  
  dReal _z[NUM] = { l[8]/2,
		    l[8]/2,
		    l[8]/2,
		    l[8]/2,
		    l[8]/2,
		    l[8]/2,
		    l[8]/2,
		    l[8]/2,
		    l[8]/2,
		    l[8]/2,
		    l[8]/2 };
  
  memcpy(x, _x, MEM_NUM);
  memcpy(y, _y, MEM_NUM);
  memcpy(z, _z, MEM_NUM);  
  
  // 回転軸              lul-z, lul-x, lll-x, rul-z, rul-x, rll-x, lua-z, lua-y, lla-y, rua-z, rua-y, rla-y,   t-y,   t-x,   h-y
  dReal _axis_x[DOF]  = { 0.00, -1.00, -1.00,  0.00,  1.00,  1.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  1.00,  0.00 };
  dReal _axis_y[DOF]  = { 0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00, -1.00, -1.00,  0.00,  1.00,  1.00,  1.00,  0.00,  1.00 };
  dReal _axis_z[DOF]  = { 1.00,  0.00,  0.00, -1.00,  0.00,  0.00,  1.00,  0.00,  0.00, -1.00,  0.00,  0.00,  0.00,  0.00,  0.00 };
  memcpy(axis_x, _axis_x, MEM_DOF);
  memcpy(axis_y, _axis_y, MEM_DOF);
  memcpy(axis_z, _axis_z, MEM_DOF);

  // max velocity setting
  dWorldSetMaxAngularSpeed( world, 10.0 );
  
  // Create link
  Create_link(world, space);
  
  // Create joint cylinder
  Create_joint_cyli(world, space);

  
#ifndef NO_SHELL
  // Create mesh
  dMatrix3 R1, R2, R3;
  dRFromZAxis(R1, 0, -1, 0);
  dRFromAxisAndAngle(R2, 0, 1, 0, M_PI*90.0/180.0);
  dMultiply0(R3, R2, R1, 3, 3, 3);
 
  double shell_x[] = { x[1], x[3] };
  double shell_y[] = { y[1], y[3] };
  double shell_z[] = { z[1], z[3] };
  
  
  char* filenames[] = { "stl/LowerLeg_new.stl",
			"stl/LowerLeg_new.stl"
  };
  
  for(int i=0; i<SHELL_NUM; i++){
    createMeshObj( world, space, outer_shell[i].body, outer_shell[i].geom, 1.0, filenames[i]);
    dBodySetPosition( outer_shell[i].body, shell_x[i], shell_y[i], shell_z[i] );
    dBodySetRotation( outer_shell[i].body, R3 );
  }
#endif
    
  // Create a Tactile sensor
  Create_tac(world, space);
  
  // Setting Joint
  Joint_Setting(world);

}

Baby_Robot2::~Baby_Robot2(){
}

void Baby_Robot2::Create_tac( dWorldID world, dSpaceID space ){

  // tac sensor position
  dReal tac_x[TAC_NUM] = { 0.00 };
  dReal tac_y[TAC_NUM] = { y[8] };
  dReal tac_z[TAC_NUM] = { z[8] + r[8] + tac_size[2]/2.};
    
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
}

void Baby_Robot2::Create_joint_cyli( dWorldID world, dSpaceID space ){
  
  for (int i = 0; i <DOF; i++) {
    // get joint rotation matrix
    dMatrix3 R;
    dRFromZAxis(R, axis_x[i], axis_y[i], axis_z[i]);
    
    // body
    joint_cyli[i].body = dBodyCreate(world);
    dBodySetPosition(joint_cyli[i].body, anchor_x[i], anchor_y[i], anchor_z[i]);
    dBodySetRotation(joint_cyli[i].body, R);
    dMassSetZero(&mass_join[i]);
    dMassSetCylinderTotal(&mass_join[i], m_join[i], 1, joint_cyli_r, joint_cyli_r);
    dBodySetMass(joint_cyli[i].body, &mass_join[i]);
 
    // geom
    joint_cyli[i].geom = dCreateCylinder(space, joint_cyli_r, joint_cyli_r);
    dGeomSetBody(joint_cyli[i].geom, joint_cyli[i].body);
  }
}

void Baby_Robot2::Create_link( dWorldID world, dSpaceID space ){

  // リンク初期姿勢
  dReal link_axis_x[NUM] = {0.00,  0.00,  0.00,  0.00,  1.00,  1.00, -1.00, -1.00,  0.00,  0.00, 0.00};
  dReal link_axis_y[NUM] = {1.00,  1.00,  1.00,  1.00,  0.00,  0.00,  0.00,  0.00,  1.00,  1.00, 1.00};
  dReal link_axis_z[NUM] = {0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00, 0.00};
  
  for (int i = 0; i <NUM; i++) {
    // get link rotation matrix
    dMatrix3 R;
    dRFromZAxis(R, link_axis_x[i], link_axis_y[i], link_axis_z[i]);
    
    // body
    link[i].body = dBodyCreate(world);
    dBodySetPosition(link[i].body, x[i], y[i], z[i]);
    dBodySetRotation(link[i].body, R);
    dMassSetZero(&mass_link[i]);
    dMassSetCappedCylinderTotal(&mass_link[i], m_link[i], 3, r[i], l[i]);
    dBodySetMass(link[i].body, &mass_link[i]);
    
    // geom
    link[i].geom = dCreateCapsule(space, r[i], l[i]);
    dGeomSetBody(link[i].geom, link[i].body);
  }
}
void Baby_Robot2::Joint_Setting( dWorldID world ){  
  for(int j=0; j<DOF; j++) joint[j] = dJointCreateHinge(world, 0);
  // fixjointは不安定
  for(int j=0; j<6; j++) body_join_fix[j] = dJointCreateFixed(world, 0);
  for(int j=0; j<4; j++) limb_join_fix[j] = dJointCreateFixed(world, 0);

#ifndef NO_SHELL
  for(int j=0; j<SHELL_NUM; j++) outer_shell_fix[j] = dJointCreateFixed(world, 0);
#endif

  // left leg to lower trunk
  dJointAttach(limb_join_fix[0],       link[1].body,  joint_cyli[2].body);
  dJointAttach(        joint[2], joint_cyli[2].body,        link[0].body); // lll-x
  dJointAttach(        joint[0],       link[0].body,  joint_cyli[0].body); // lul-z
  dJointAttach(        joint[1], joint_cyli[0].body,  joint_cyli[1].body); // lul-x
  dJointAttach(body_join_fix[0], joint_cyli[1].body,        link[9].body);
  // right leg to lower trunk
  dJointAttach(limb_join_fix[1],       link[3].body,  joint_cyli[5].body);
  dJointAttach(        joint[5], joint_cyli[5].body,        link[2].body); // rll-x
  dJointAttach(        joint[3],       link[2].body,  joint_cyli[3].body); // rul-z
  dJointAttach(        joint[4], joint_cyli[3].body,  joint_cyli[4].body); // rul-x
  dJointAttach(body_join_fix[1], joint_cyli[4].body,        link[9].body);
  // left arm to upper trunk
  dJointAttach(limb_join_fix[2],       link[5].body,  joint_cyli[8].body);
  dJointAttach(        joint[8], joint_cyli[8].body,        link[4].body);  
  dJointAttach(        joint[6],       link[4].body,  joint_cyli[6].body);
  dJointAttach(        joint[7], joint_cyli[6].body,  joint_cyli[7].body);
  dJointAttach(body_join_fix[2], joint_cyli[7].body,        link[8].body);
  // right arm to upper trunk
  dJointAttach(limb_join_fix[3],       link[7].body,  joint_cyli[11].body);
  dJointAttach(        joint[11],joint_cyli[11].body,        link[6].body);
  dJointAttach(        joint[9],      link[6].body,  joint_cyli[9].body);
  dJointAttach(        joint[10], joint_cyli[9].body,  joint_cyli[10].body);
  dJointAttach(body_join_fix[3], joint_cyli[10].body,        link[8].body);
  // upper trunk to lower trunk
  dJointAttach(        joint[12],      link[8].body,  joint_cyli[12].body); // t-y
  dJointAttach(        joint[13],joint_cyli[12].body,  joint_cyli[13].body); // t-x
  dJointAttach(body_join_fix[4], joint_cyli[13].body,        link[9].body);
  // head to upper trunk
  dJointAttach(       joint[14],        link[8].body, joint_cyli[14].body); // h-y
  dJointAttach(body_join_fix[5], joint_cyli[14].body,        link[10].body);

#ifndef NO_SHELL
  // outer shell to body
  dJointAttach( outer_shell_fix[0], link[1].body, outer_shell[0].body);
  dJointAttach( outer_shell_fix[1], link[3].body, outer_shell[1].body);
#endif
  
  for(int j=0; j<DOF; j++){
      dJointSetHingeAnchor(joint[j], anchor_x[j], anchor_y[j],anchor_z[j]);
      dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]);
  }
  for(int j=0; j<6; j++) dJointSetFixed(body_join_fix[j]);
  for(int j=0; j<4; j++) dJointSetFixed(limb_join_fix[j]);

#ifndef NO_SHELL
  for(int j=0; j<SHELL_NUM; j++) dJointSetFixed(outer_shell_fix[j]);
#endif

  // Fix tactile sensor to link
  for (int j=0; j<TAC_NUM; j++){
    tac_fix[j] = dJointCreateFixed(world, 0);
    dJointAttach(tac_fix[j], link[8].body, tac_sensor[j].body);
    dJointSetFixed(tac_fix[j]);
  }

  // Joint param setting
  /*
  for (int i=0; i<DOF; i++){
    dJointSetHingeParam(joint[i], dParamLoStop, ROM[2*i]/180.*M_PI);
    dJointSetHingeParam(joint[i], dParamHiStop, ROM[2*i+1]/180.*M_PI);
  }
  */

  for (int i=0; i<6; i++){
    dJointSetFixedParam(body_join_fix[i], dParamFMax, 0.1);
    dJointSetFixedParam(body_join_fix[i], dParamLoStop, -1/180.*M_PI);
    dJointSetFixedParam(body_join_fix[i], dParamHiStop,  1/180.*M_PI);
  }
  
  for (int i=0; i<4; i++){
    dJointSetFixedParam(limb_join_fix[i], dParamFMax, 0.1);
    dJointSetFixedParam(limb_join_fix[i], dParamLoStop, -1/180.*M_PI);
    dJointSetFixedParam(limb_join_fix[i], dParamHiStop,  1/180.*M_PI);
  }

#ifndef NO_SHELL
  for (int i=0; i<SHELL_NUM; i++){
    dJointSetFixedParam(outer_shell_fix[i], dParamFMax, 0.1);
    dJointSetFixedParam(outer_shell_fix[i], dParamLoStop, -1/180.*M_PI);
    dJointSetFixedParam(outer_shell_fix[i], dParamHiStop,  1/180.*M_PI);
  }
#endif

}

void Baby_Robot2::restrict_angle(int i){
  ANGLE[i] = std::max(ROM[2*i], ANGLE[i]);
  ANGLE[i] = std::min(ROM[2*i+1], ANGLE[i]);
}


void Baby_Robot2::control() {
  /***  PD  ****/
  static long int step = 0;
  static dReal z[3*DOF] = {0};
  // k1:比例ゲイン,  fMax：最大トルク[Nm]
  dReal k1 =  0.2, k2 = 0.00, k3 = 0.1,  fMax  = 10.0;
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

bool Baby_Robot2::Is_Tactile(dGeomID obj){
  for(int i=0; i<TAC_NUM; i++)
    if(tac_sensor[i].geom == obj) return true;
  else return false;
}

int Baby_Robot2::Which_Tactile(dGeomID obj){
  for(int i=0; i<TAC_NUM; i++)
    if(tac_sensor[i].geom == obj) return i;
  else return -1;
}

bool Baby_Robot2::BodyTactileCollision(dGeomID obj1, dGeomID obj2){
  for(int i=0; i<TAC_NUM; i++)
    for(int j=0; j<NUM; j++)
      if(tac_sensor[i].geom == obj1 && link[j].geom == obj2
	 ||
	 tac_sensor[i].geom == obj2 && link[j].geom == obj1)
	return true;
  return false;
}


void Baby_Robot2::command(int cmd) {
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
  case 'a': ANGLE[11] += 5; restrict_angle(11); break;
  case 's': ANGLE[12] += 5; restrict_angle(12); break; 
  case 'd': ANGLE[13] += 5; restrict_angle(13); break; 
  case 'f': ANGLE[14] += 5; restrict_angle(14); break;
    
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
  case 'z': ANGLE[11] -= 5; restrict_angle(11); break;
  case 'x': ANGLE[12] -= 5; restrict_angle(12); break; 
  case 'c': ANGLE[13] -= 5; restrict_angle(13); break; 
  case 'v': ANGLE[14] -= 5; restrict_angle(14); break; 
  }
}


Baby_Robot2* robot;
