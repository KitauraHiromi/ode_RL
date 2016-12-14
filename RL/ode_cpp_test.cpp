#include <ode/ode.h>
#include <ode/odecpp_collision.h>
#include <ode/odecpp.h>
#include <drawstuff/drawstuff.h>
#include <vector>
#include <string>

namespace
{
  const std::string DrawstuffTexturePath = "/Users/kumada/BuildSpace/ode-0.12/drawstuff/textures";

  // camera position
  float xyz[] = {0, -3, 1};

  // camera direction(heading, pitch and roll numbers in degrees)
  float hpr[] = {90, 0, 0};

  dWorld world;
  dHashSpace space;
  dJointGroup joint_group;

  // ground
  dPlane plane;

  // A ball consists of a rigid body and a geometry.
          struct Ball
	  {
	    dBody body;
	    dSphere geom;
	  };
  const int NumOfBalls = 10;
  std::vector<Ball> balls(NumOfBalls);

  // radius of a ball
  const dReal radius = 0.2;

  // the maximum number of contact points
  const int MaxNumOfContacts = 10;
  std::vector<dContact> contacts(MaxNumOfContacts);

  bool flag = false;
  void near_callback(void* data, dGeomID o1, dGeomID o2)
  {
    if ( o1 == plane.id() || o2 == plane.id() ) {
      int n = dCollide(o1, o2, MaxNumOfContacts, &contacts[0].geom, sizeof(dContact));
      if ( n > 0 ) {
	flag = true;
      } else {
	flag = false;
      }
      for ( int i = 0; i < n; ++i ) {
	dContact& contact = contacts[i];
	contact.surface.mode = dContactBounce;
	contact.surface.mu = dInfinity;
	contact.surface.bounce = 0.9;
	contact.surface.bounce_vel = 0.0;
	dContactJoint joint;
	joint.create(world.id(), joint_group.id(), &contact);
	joint.attach(dGeomGetBody(contact.geom.g1), dGeomGetBody(contact.geom.g2));
      }
    }
  }
}

int main(int argc, char** argv)
{
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = [](){
    dsSetViewpoint(xyz, hpr);
    dsSetSphereQuality(3);
  };
  fn.step = [](int){
    flag = false;
    joint_group.empty();
    space.collide(0, &near_callback);
    world.step(0.01);

    if ( !flag ) {
      dsSetColor(1, 0, 0);
    } else {
      dsSetColor(0, 0, 1);
    }
    for ( auto& ball : balls ) {
      const dReal* pos = ball.body.getPosition();
      const dReal* rot = ball.body.getRotation();
      //dsDrawSphereD(pos, rot, radius);
    }
  };
  fn.command = 0;
  fn.stop = 0;
  fn.path_to_textures = DrawstuffTexturePath.c_str();

  dInitODE();

  const dReal gravity[] = {0, 0, -0.5};
  world.setGravity(gravity);

  // create a ground
  // ax + by + cz = d
  // normal vector: (a, b, c) = (0, 0, 1)
  plane.create(space.id(), 0, 0, 1, 0);

  dMass m;
  m.setZero();
  const dReal mass = 1.0;
  m.setSphereTotal(mass, radius);
  const dReal initial_position[] = {0, 0, 2};

  for ( std::size_t i = 0, n = balls.size(); i < n; ++i ) {
    Ball& ball = balls[i];
    ball.body.create(world.id());
    ball.body.setMass(&m);
    ball.body.setPosition(
			  initial_position[0] + 0.4 * i - 0.4 * (n - 1) * 0.5,
			  initial_position[1],
			                          initial_position[2]
			  );
    // The geometry(ball.geom) is associated with the rigid body(ball.body).
    ball.geom.create(space.id(), radius);
    ball.geom.setBody(ball.body.id());
  }

  dsSimulationLoop(argc, argv, 640, 480, &fn);
  dCloseODE();
  return 0;
}
