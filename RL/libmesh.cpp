#include "include/libmesh.hpp"

ode_utils::StlReader<float,dTriIndex> *mesh;

void createMeshObj( dWorldID world_, dSpaceID space_, dBodyID &body_, dGeomID &geom_, dReal mass_, char* filename)
{

  mesh = new ode_utils::StlReader<float,dTriIndex>(filename);

  body_ = dBodyCreate( world_ );

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

void drawMesh(dGeomID geom_){
  const dReal* pos = dGeomGetPosition(geom_);
  const dReal* rot = dGeomGetRotation(geom_);
  
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
    dsDrawTriangleD( pos, rot, &v[0], &v[3], &v[6], 1 );
  }
}
