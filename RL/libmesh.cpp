#include "include/libmesh.hpp"

std::vector< ode_utils::StlReader<float,dTriIndex>* > mesh;
std::map< dGeomID, int > dict;

void createMeshObj( dWorldID world_, dSpaceID space_, dBodyID &body_, dGeomID &geom_, dReal mass_, char* filename)
{
  int n = mesh.size();
  mesh.push_back( new ode_utils::StlReader<float,dTriIndex>(filename) );
  printf ("%s\n", filename);

  body_ = dBodyCreate( world_ );

  dTriMeshDataID data;
  data = dGeomTriMeshDataCreate();
  dGeomTriMeshDataBuildSingle(data, mesh[n]->getVertices(), 3*sizeof(float), mesh[n]->getVertexCount(),
			      mesh[n]->getIndices(), mesh[n]->getIndexCount(), 3*sizeof(dTriIndex));

  geom_ = dCreateTriMesh(space_, data, 0, 0, 0);
  dGeomSetData( geom_, data );

  dMass m;
  dMassSetTrimesh( &m, mass_, geom_ );
  dGeomSetPosition( geom_, -m.c[0], -m.c[1], -m.c[2] );
  dMassTranslate( &m, -m.c[0], -m.c[1], -m.c[2] );
  dBodySetMass( body_, &m );

  dGeomSetBody( geom_, body_ );

  dict[geom_] = n;
}

void drawMesh(dGeomID geom_){
  const dReal* pos = dGeomGetPosition(geom_);
  const dReal* rot = dGeomGetRotation(geom_);
  int n = dict[geom_];
  
  for (int ii = 0; ii < mesh[n]->getIndexCount()/3; ii++) {
    const dReal v[9] = {
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+0]*3 + 0],
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+0]*3 + 1],
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+0]*3 + 2],
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+1]*3 + 0],
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+1]*3 + 1],
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+1]*3 + 2],
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+2]*3 + 0],
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+2]*3 + 1],
      mesh[n]->getVertices()[mesh[n]->getIndices()[ii*3+2]*3 + 2]
    };
    dsDrawTriangleD( pos, rot, &v[0], &v[3], &v[6], 1 );
  }
}
