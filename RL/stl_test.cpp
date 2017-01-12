#include <dlfcn.h>
#include <ode/ode.h>
#include "include/libmesh.hpp"
#include <drawstuff/drawstuff.h>
#include <iostream>
bool __VIEW__ = false;

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "../../drawstuff/textures"
#endif

int WindowWidth  = 480; //ウィンドウの幅
int WindowHeight = 320; //ウィンドウの高さ

#ifdef dDOUBLE
    #define dsDrawBox dsDrawBoxD
    #define dsDrawTriangle dsDrawTriangleD
#endif

dBodyID body1, body2, body3;
dGeomID geom1, geom2, geom3;



// 衝突検出用関数
void nearCallback( void *data, dGeomID o1, dGeomID o2 ){
  const int MAX_CONTACTS = 500;

  dBodyID b1 = dGeomGetBody( o1 ); // 物体1
  dBodyID b2 = dGeomGetBody( o2 ); // 物体2

  if ( b1 && b2 && dAreConnectedExcluding( b1, b2, dJointTypeContact ) )
    return; // 衝突対象でない物体の衝突ははずす

  dContact contact[MAX_CONTACTS];
  for ( int i=0; i<MAX_CONTACTS; i++ ){
    // 物体同士の接触時のパラメータ設定
    contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP;
    contact[i].surface.mu = 100.0;   // 摩擦係数
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.2;     // 反発係数
    contact[i].surface.bounce_vel = 0.01;
    contact[i].surface.soft_cfm = 0.01;  // CFM設定
    contact[i].surface.soft_erp = 0.5;
  }

  // 衝突検出
  int numc = dCollide( o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof( dContact ) );
  if ( numc > 0 ){
    for ( int i=0; i<numc; i++ ){
      // 衝突の発生
      dJointID c = dJointCreateContact( world, contactgroup, contact+i );
      dJointAttach( c, b1, b2 );
    }
  }
}

// start simulation - set viewpoint
void start()
{
  float xyz[3] = { 0.f, 5.f, 2.f };
  float hpr[3] = { -90.f, -10.f, 0.f };

  dsSetViewpoint( xyz, hpr );
}

  
// simulation loop
void simLoop( int pause ){
  if (!pause){
    dSpaceCollide( space, 0, &nearCallback );
    dWorldStep( world, 0.01 );
    dJointGroupEmpty( contactgroup );
  }
  if(__VIEW__){
    dsSetColor( 1, 1, 0 );
    
    drawMesh(geom1);
    drawMesh(geom2);
  }
}



int main(int argc, char* argv[]){
  std::cout << "flag1" << std::endl;
  dInitODE();

  char filename[] = "stl/20150622_LowerLeg_w004.stl";
  printf("%d\n", argc);
  if( argc > 1 ){
    __VIEW__ = true;
  }
  
  // create world
  world = dWorldCreate();
  space = dHashSpaceCreate( 0 );
  contactgroup = dJointGroupCreate( 0 );
  dCreatePlane( space, 0, 0, 1, 0 );
  dWorldSetCFM (world,1e-5);
  dWorldSetERP (world,0.1);
  dWorldSetGravity( world, 0.0, 0.0, -9.8 );
  //dWorldSetLinearDamping(world,0.001);

  std::cout << "flag2" << std::endl;
  
  dMatrix3 R;
  dRFromAxisAndAngle( R, 1, 0, 0, 90.0);
  
  // create MeshObj
  createMeshObj( world, space, body1, geom1, 1.0, filename );
  createMeshObj( world, space, body2, geom2, 1.0, filename );
  dBodySetPosition( body1, 0.0, 0.0, 1.0 );
  dBodySetPosition( body2, 0.0, 0.0, 2.0 );
  dBodySetRotation( body1, R);
  dBodySetRotation( body2, R);

  if(__VIEW__){
      dsFunctions fn;
      fn.version = DS_VERSION;
      fn.start   = &start;
      fn.step    = &simLoop;
      fn.command = 0;
      fn.stop    = 0;
      fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

      dsSimulationLoop (argc,argv,WindowWidth,WindowHeight,&fn);
  }else{
    while(1){
    static unsigned int count = 0;
    simLoop(0);
    printf("%d\n", ++count);
  }
  }
  dJointGroupDestroy( contactgroup );
  dSpaceDestroy( space );
  dWorldDestroy( world );
  
  return 1;
}
