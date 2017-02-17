#include "include/tactile_sensor.hpp"

#define __USE_TACTILE__
#define TAC_AMP 1000.0

Tac_Sheet::Tac_Sheet(dWorldID _world, dSpaceID _space, dBodyID _fix_body, int _sheet_num, dReal _cx, dReal _cy, dReal _cz, dReal _r, int _rn, int _zn){

  // place tactile sensors along with cilinder surface
  sheet_num = _sheet_num;
  center[0] = _cx; center[1] = _cy; center[2] = _cz;
  range = 0.005;
  r = _r - range/2.;
  round_num = _rn;
  axis_num = _zn;
  total_num = round_num * axis_num;
  fix_body = _fix_body;
  tac_sensors.resize(total_num);
  for (int i=0; i<total_num; i++){
    tac_sensors[i].value[0] = -1;
    tac_sensors[i].value[1] = -1;
    tac_sensors[i].value[2] = -1;
  }
  
  std::vector<dReal> tac_x(total_num);
  std::vector<dReal> tac_y(total_num);
  std::vector<dReal> tac_z(total_num);
  dMatrix3 tac_R[total_num];
  //std::vector< std::array<dReal, 12> > tac_R;
  //tac_R.resize(total_num);
  
  for (int i=0; i<axis_num; i++){
    for (int j=0; j<round_num; j++){
      dReal sin = SIN(double(j)/round_num * 2*M_PI);
      dReal cos = COS(double(j)/round_num * 2*M_PI);
      tac_y[round_num*i + j] = r*sin;
      tac_z[round_num*i + j] = 0.025*(i-5);
      tac_x[round_num*i + j] = r*cos;
      dRFromZAxis(tac_R[round_num*i + j], cos, sin, 0);
    }
  }
  
  for (int i=0; i<total_num; i++){
    tac_sensors[i].geom = dCreateRay(_space, range);
    dGeomSetBody(tac_sensors[i].geom, fix_body);
    dGeomSetOffsetPosition(tac_sensors[i].geom, tac_x[i], tac_y[i], tac_z[i]);
    dGeomSetOffsetRotation(tac_sensors[i].geom, tac_R[i]);
  }
}

Tac_Sheet::~Tac_Sheet(){
}

void Tac_Sheet::Get_Tactile_Values(std::vector<dReal> &values){
  for (int i=0; i<total_num; i++){
    values[i] = Convert_Tactile_Value(tac_sensors[i].value[0]);
  }
}

void Tac_Sheet::Set_Tactile_Values(){
  for (int i=0; i<total_num; i++){
    tac_sensors[i].value[1] = Convert_Tactile_Value(tac_sensors[i].value[0]);
  } 
}

void Tac_Sheet::Set_Dist(int n, dReal dist){
  tac_sensors[n].value[0] = dist;
}

bool Tac_Sheet::In_Sheet(dGeomID _geom){
  for (int i=0; i<total_num; i++){
    if (tac_sensors[i].geom == _geom){
      return true;
    }
  }
  return false;
}

int Tac_Sheet::Which_Tactile(dGeomID _geom){
    for (int i=0; i<total_num; i++){
    if (tac_sensors[i].geom == _geom){
      return i;
    }
  }
  return -1;
}

void Tac_Sheet::Draw_Sheet(){
  for(int i=0; i<total_num; i++){
    dsSetColor(1, 0, 0);
    dVector3 Origin, Direction;
    dGeomRayGet(tac_sensors[i].geom, Origin, Direction);
    dReal Length = dGeomRayGetLength(tac_sensors[i].geom);

    dVector3 End;
    End[0] = Origin[0] + (Direction[0] * Length);
    End[1] = Origin[1] + (Direction[1] * Length);
    End[2] = Origin[2] + (Direction[2] * Length);
    End[3] = Origin[3] + (Direction[3] * Length);

    dsDrawLineD(Origin, End);
  }
}

void Tac_Sheet::Write_Data(std::ofstream& tac_out){
  // todo implement timestamp
  tac_out << sheet_num << " no_timestamp " << round_num << ' ' << axis_num;
  for(int i=0; i<total_num; i++){
    tac_out << tac_sensors[i].value[1] << ' ';
  }
  tac_out << std::endl;
}

bool Tac_Sheet::Callback(dGeomID o1, dGeomID o2){
  // 埋もれた場合にどうなるかは未検証
  int n1 = Which_Tactile(o1);
  int n2 = Which_Tactile(o2);
  int n = std::max(n1, n2);
  if( n > 0 ){
    dContactGeom c;
    int numc = dCollide( o1, o2, 1, &c, sizeof( dContactGeom) );
    if (numc > 0){
      tac_sensors[n].value[0] = c.depth;
    }
    return true;
  }
  return false;
}

dReal Tac_Sheet::Convert_Tactile_Value(dReal dist){
  if( dist >= 0.0001 ){
    return std::min( 1.0/(dist+0.1), TAC_AMP)/TAC_AMP;
  }else{
    return 0;
  }
}
