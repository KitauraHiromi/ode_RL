#include <ode/ode.h>
#include <drawstuff/drawstuff.h>


#ifdef dDOUBLE
  #define dsDrawBox dsDrawBoxD
  #define dsDrawSphere dsDrawSphereD
  #define dsDrawLine dsDrawLineD
#endif

#define MAX_CONTACTS 10 // 最大の衝突検出可能数
#define RAY_NUM 3000

static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

static dBodyID body_box;
static dGeomID geom_box;
static dGeomID geom_ray[RAY_NUM];

static dBodyID body_ball;
static dGeomID geom_ball;

dReal  dist; // rayに接触した物体の距離を保持


// 衝突検出用関数
static void nearCallback( void *data, dGeomID o1, dGeomID o2 )
{
  dBodyID b1 = dGeomGetBody( o1 ); // 物体1
  dBodyID b2 = dGeomGetBody( o2 ); // 物体2

  if ( b1 && b2 && dAreConnectedExcluding( b1, b2, dJointTypeContact ) )
    return; // 衝突対象でない物体の衝突ははずす

  //////////////////////////////////////////////////////////
  // 距離を取得
  for(int i=0; i<RAY_NUM; i++){
    if (o1==geom_ray[i] || o2==geom_ray[i]) {
      dContactGeom c;
      int numc = dCollide( o1, o2, 1, &c, sizeof( dContactGeom ) );
      if (numc > 0) {
	dist = c.depth; // depthはrayの始点位置からの距離
      }
      return;
    }
  }
  //////////////////////////////////////////////////////////

  dContact contact[MAX_CONTACTS];
  for ( int i=0; i<MAX_CONTACTS; i++ )
    {
      // 物体同士の接触時のパラメータ設定
      contact[i].surface.mode = dContactBounce | dContactSoftERP | dContactSoftCFM;
      contact[i].surface.mu = dInfinity;     // 摩擦係数
      contact[i].surface.bounce = 0.5;       // 反発係数
      contact[i].surface.soft_erp = 0.3;     // ERP設定
      contact[i].surface.soft_cfm = 0.0001;  // CFM設定
    }

  // 衝突検出
  int numc = dCollide( o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof( dContact ) );
  if ( numc > 0 )
    {
      for ( int i=0; i<numc; i++ )
	{
	  // 衝突の発生
	  dJointID c = dJointCreateContact( world, contactgroup, contact+i );
	  // 接触ジョイント生成
	  dJointAttach( c, b1, b2 ); // 接触ジョイント接続
	}
    }
}

// start simulation - set viewpoint
static void start()
{
  static float xyz[3] = { 2.f, -10.f, 3.f };
  static float hpr[3] = { 90.f, -15.f, 0.f };

  dsSetViewpoint( xyz, hpr );
}

// simulation loop
static void simLoop( int pause )
{
  dist = -1.0;

  // Ctl+p が押されたらifに入らない
  if (!pause)
    {
      dSpaceCollide( space, 0, &nearCallback ); // 衝突検出

      dWorldStep( world, 0.01 );

      dJointGroupEmpty( contactgroup );
    }

  ///////////////////////////////////////////////////
  // 距離表示
  if (dist>=0.0) {
    printf( "dist = %f\n", dist );
  }
  ///////////////////////////////////////////////////

  // draw box
  {
    dsSetColor( 1.0f, 1.0f, 0.0f );
    dVector3 size;
    dGeomBoxGetLengths( geom_box, size );
    dsDrawBox( dGeomGetPosition( geom_box ), dGeomGetRotation( geom_box ), size );
  }
  // draw ray
  {
    for(int i=0; i<1; i++){
      dsSetColor( 1.0f, 0.0f, 0.0f );
      
      dVector3 Origin, Direction;
      dGeomRayGet(geom_ray[i], Origin, Direction);
      dReal Length = dGeomRayGetLength(geom_ray[i]);
      
      dVector3 End;
      End[0] = Origin[0] + (Direction[0] * Length);
      End[1] = Origin[1] + (Direction[1] * Length);
      End[2] = Origin[2] + (Direction[2] * Length);
      End[3] = Origin[3] + (Direction[3] * Length);
      
      dsDrawLine(Origin, End);
    }
  }
  // draw ball
  {
    dsSetColor( 0.0f, 1.0f, 1.0f );
    dReal radius = dGeomSphereGetRadius( geom_ball );
    dsDrawSphere( dGeomGetPosition( geom_ball ), dGeomGetRotation( geom_ball ), radius );
  }
}


int main( int argc, char* argv[] )
{
  dInitODE();

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = 0;
  fn.stop    = 0;
  fn.path_to_textures = "../../drawstuff/textures";

  world = dWorldCreate();
  dWorldSetGravity( world, 0.0, 0.0, -9.8 );

  space = dHashSpaceCreate( 0 );
  contactgroup = dJointGroupCreate( 0 );

  // plane create
  {
    dCreatePlane( space, 0, 0, 1, 0 );
  }

  // box creating
  {
    dReal pos[3] = { 0.0, 0.0, 1.5 };
    dReal size[3] = { 2.0, 2.0, 2.0 };

    // body setting
    body_box = dBodyCreate( world );
    dBodySetPosition( body_box, pos[0], pos[1], pos[2] );
    // geom setting
    geom_box = dCreateBox( space, size[0], size[1], size[2] );
    dGeomSetBody( geom_box, body_box );
    // mass setting
    dMass mass;
    dMassSetBoxTotal( &mass, 10.0, size[0], size[1], size[2] ); // 10kg
    dBodySetMass( body_box, &mass );
    // rotation
    dMatrix3 R;
    dRFromAxisAndAngle( R, 1.0, 0.0, 0.0, 0.0*M_PI/180 );
    dBodySetRotation( body_box, R );

    //// fixed joint setting
    //dJointID fixed;
    //fixed = dJointCreateFixed( world, 0 );
    //dJointAttach( fixed, NULL, body_box );
    //dJointSetFixed( fixed );
  }

  // Ray setting
  {
    dReal pos[3] = { 1.0, 0.0, 0.0 };
    dReal length = 5.0;

    for(int i=0; i<RAY_NUM; i++){
      geom_ray[i] = dCreateRay( space, length );
      dGeomSetBody( geom_ray[i], body_box );  // <= body is box
    
      // position and rotation setting
      dGeomSetOffsetPosition( geom_ray[i], pos[0]+0.1*(i/100), pos[1], pos[2]+0.1*(i%100) );
      dMatrix3 R;
      dRFromAxisAndAngle( R, 0.0, 1.0, 0.0, 90.0*M_PI/180 );
      dGeomSetOffsetRotation( geom_ray[i], R );
      //dGeomRaySet( geom_ray, pos[0], pos[1], pos[2], dir[0], dir[1], dir[2] );
    }
  }

  // ball creating
  {
    dReal pos[3] = { 10.0, 0.0, 1.0 };
    dReal radius = 1.0;

    // body setting
    body_ball = dBodyCreate( world );
    dBodySetPosition( body_ball, pos[0], pos[1], pos[2] );
    // geom setting
    geom_ball = dCreateSphere( space, radius );
    dGeomSetBody( geom_ball, body_ball );
    // mass setting
    dMass mass;
    dMassSetSphereTotal( &mass, 1.0, radius ); // 10kg
    dBodySetMass( body_ball, &mass );
    // rotation
    dMatrix3 R;
    dRFromAxisAndAngle( R, 1.0, 0.0, 0.0, 0.0*M_PI/180 );
    dBodySetRotation( body_ball, R );
  }

  //ボールを転がす
  dBodyAddForce( body_ball, -500.0, 0.0, 0.0 );


  dsSimulationLoop( argc, argv, 320, 240, &fn );

  dWorldDestroy( world );
  dCloseODE();
  return 0;
}
