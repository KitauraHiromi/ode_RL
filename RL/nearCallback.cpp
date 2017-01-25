#include "include/nearCallback.hpp"

void ContactSetting(dContact* contact, double GKD, double GKP, double bounce, double bounce_vel)
{
  // 衝突状態フラグの設定
  contact->surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
  // クーロン摩擦
  contact->surface.mu = 0.5;
  // 地面の柔軟性設定
  contact->surface.soft_erp = DT * GKP / ( DT * GKP + GKD);
  contact->surface.soft_cfm = 1 / ( DT * GKP + GKD);    
  // 反発係数, 反発最小速度
  contact->surface.bounce     = bounce; // (0.0~1.0) restitution parameter
  contact->surface.bounce_vel = bounce_vel; // minimum incoming velocity for bounce
}

void nearCallback(void *data, dGeomID o1, dGeomID o2)
{ 
  // 二物体が拘束されていれば衝突判定をしない
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  
  if(b1 && b2 && dAreConnected(b1, b2)) return;

#ifndef __NO_TACTILE__
  // 触覚センサとbodyは衝突判定しない
  if(b1 && b2 && robot->BodyTactileCollision(o1, o2)) return;
#endif

#ifndef NO_SHELL
  if(b1 && b2 && robot->BodyBodyCollision(o1, o2)) return;
#endif
  
  if(b1 && b2 && robot->BodyJointCollision(o1, o2)) return;

  
  // 二物体の衝突状態
  const int N = 30;
  dContact contact[N];
  int n = dCollide (o1, o2, N, &contact[0].geom, sizeof(dContact));

  if (n > 0) {
    for (int i=0; i<n; i++) {
      // if(o1==robot->tac_sensor.geom || o2==robot->tac_sensor.geom){
#ifndef __NO_TACTILE__
      if(robot->Is_Tactile(o1) || robot->Is_Tactile(o2)){
	ContactSetting(&contact[i], softGKD, softGKP, 0.1, 0);
      }
      else{
#endif
      ContactSetting(&contact[i], hardGKD, hardGKP, 0.4, 0);
#ifndef __NO_TACTILE__
      }
#endif
      
      dJointID c = dJointCreateContact(WORLD_ID, world.contactgroup, &contact[i]);
      dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));

      // Tactile value setting
#ifndef __NO_TACTILE__
      int j = std::max(robot->Which_Tactile(o1), robot->Which_Tactile(o2));
      if(j > -1){
	// contact_data is world coordinate.
	dJointSetFeedback(c, world.contact_data);
	world.contact_data = dJointGetFeedback(c);
	//convert contact_data to relative coordinate.
	dBodyVectorFromWorld(robot->tac_sensor[j].body,
			     world.contact_data->f1[0],
			     world.contact_data->f1[1],
			     world.contact_data->f1[2],
			     robot->tac_sensor[j].value);
	
      }
#endif
      // 衝突点に反作用力を付加
      
      dBodyAddForceAtPos(b1,
			 0.1,
			 0,
			 0,
			 contact[i].geom.pos[0],
			 contact[i].geom.pos[1],
			 contact[i].geom.pos[2]); 
      
    }
  }
}
