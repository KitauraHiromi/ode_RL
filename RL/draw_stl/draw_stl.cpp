#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "StlReader.h"

static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

static dBodyID body;
static dGeomID geom; 

ode_utils::StlReader<float, dTriIndex> *mesh;

void createMeshObj( dWorldID world_, dSpaceID space_, dBodyID &body_, dGeomID &geom_, dReal mass_)
{
 
    body_ = dBodyCreate( world );
 
    dTriMeshDataID data;
    data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(data, mesh->getVertices(), 3*sizeof(float), mesh->getVertexCount(),
        mesh->getIndices(), mesh->getIndexCount(), 3*sizeof(dTriIndex));
 
    geom_ = dCreateTriMesh(space_, data, 0, 0, 0);
    dGeomSetData( geom_, data );
 
    dMass m;
    dMassSetTrimesh( &m, mass_, geom_ );
    dGeomSetPosition( geom_, -m.c[0], -m.c[1], -m.c[2] );
    dMassTranslate( &m, -m.c[0], -m.c[1], -m.c[2] );
    dBodySetMass( body_, &m );
 
    dGeomSetBody( geom_, body_ );
}


// 衝突検出用関数
void nearCallback( void *data, dGeomID o1, dGeomID o2 )
{
	const int MAX_CONTACTS = 10;

	dBodyID b1 = dGeomGetBody( o1 ); // 物体1
	dBodyID b2 = dGeomGetBody( o2 ); // 物体2

	if ( b1 && b2 && dAreConnectedExcluding( b1, b2, dJointTypeContact ) )
	return; // 衝突対象でない物体の衝突ははずす

	dContact contact[MAX_CONTACTS];
	for ( int i=0; i<MAX_CONTACTS; i++ )
	  {
	    // 物体同士の接触時のパラメータ設定
	    contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP;
	    contact[i].surface.mu = dInfinity;   // 摩擦係数
	    contact[i].surface.mu2 = 0;
	    contact[i].surface.bounce = 0.5;     // 反発係数
	    contact[i].surface.bounce_vel = 0.1;
	    contact[i].surface.soft_cfm = 0.0001;  // CFM設定
	    contact[i].surface.soft_erp = 0.1;
	  }
	
	// 衝突検出
	int numc = dCollide( o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof( dContact ) );
	if ( numc > 0 )
	  {
	    for ( int i=0; i<numc; i++ )
	      {
		// 衝突の発生
		dJointID c = dJointCreateContact( world, contactgroup, contact+i );
		dJointAttach( c, b1, b2 );
	      }
	  }
}



// start simulation - set viewpoint
void start()
{
  float xyz[3] = { 0., -5., 1. };
  float hpr[3] = { 90., 15., 0. };
  
  //dsSetViewpoint( xyz, hpr );
}

// simulation loop
void simLoop( int pause )
{
  
  if (!pause)
    {
      dSpaceCollide( space, 0, &nearCallback );
      
      dWorldStep( world, 0.01 );
      
      dJointGroupEmpty( contactgroup );
    }
  
  
  dsSetColor( 1, 1, 0 );
  
  
  {
    const dReal* pos = dGeomGetPosition(geom);
    const dReal* rot = dGeomGetRotation(geom);
    
    for (int ii = 0; ii < mesh->getIndexCount()/3; ii++) {
      const dReal v[9] = {
	mesh->getVertices()[mesh->getIndices()[ii*3+0]*3 + 0],
	mesh->getVertices()[mesh->getIndices()[ii*3+0]*3 + 1],
	mesh->getVertices()[mesh->getIndices()[ii*3+0]*3 + 2],
	mesh->getVertices()[mesh->getIndices()[ii*3+1]*3 + 0],
	mesh->getVertices()[mesh->getIndices()[ii*3+1]*3 + 1],
	mesh->getVertices()[mesh->getIndices()[ii*3+1]*3 + 2],
	mesh->getVertices()[mesh->getIndices()[ii*3+2]*3 + 0],
	mesh->getVertices()[mesh->getIndices()[ii*3+2]*3 + 1],
	mesh->getVertices()[mesh->getIndices()[ii*3+2]*3 + 2]
      };
      //dsDrawTriangleD( pos, rot, &v[0], &v[3], &v[6], 1 );
    }
    
  }
  
}


int main( int argc, char* argv[] )
{
  
  // setup pointers to drawstuff callback functions
  
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = 0;
  fn.stop    = 0;
  fn.path_to_textures = "../../../drawstuff/textures";
  
  dInitODE();
  
  // create world
  world = dWorldCreate();
  space = dHashSpaceCreate( 0 );
  contactgroup = dJointGroupCreate( 0 );
  dCreatePlane( space, 0, 0, 1, 0 );
  dWorldSetCFM (world,1e-5);
  dWorldSetERP (world,0.1);
  dWorldSetGravity( world, 0.0, 0.0, -9.8 );
  
  
  mesh = new ode_utils::StlReader<float, dTriIndex>("monkey_binary.stl");
  if (!mesh->isCompleted()) {
    printf("NG\n");
    printf("msg:\n%s", mesh->message() );
    exit(1);
  }else{
    printf("stl load completed\n");
  }
  
  
  // create MeshObj
  createMeshObj( world, space, body, geom, 1.0 );
  dBodySetPosition( body, 0.0, 0.0, 3.0 );
  dMatrix3 R;
  dRFromAxisAndAngle( R, 1, 0, 0, 90.0);
  dBodySetRotation( body, R);
  
  // starting simulation
  //dsSimulationLoop( argc, argv, 320, 240, &fn );
  
  dJointGroupDestroy( contactgroup );
  dSpaceDestroy( space );
  dWorldDestroy( world );
  
  return 0;
}

