#include <karto_sdk/Karto.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

struct InspectGrid : karto::OccupancyGrid {
  using karto::OccupancyGrid::AddScan;
  int traces=0, occupied_endpoints=0;
  double clearing_length=0;
  InspectGrid():OccupancyGrid(500,500,karto::Vector2<kt_double>(-12.5,-12.5),.05){}
  kt_bool RayTrace(const karto::Vector2<kt_double>& a,
                   const karto::Vector2<kt_double>& b,
                   kt_bool hit, kt_bool update=false) override {
    ++traces; occupied_endpoints+=hit;
    if(!hit)clearing_length=std::hypot(b.GetX()-a.GetX(),b.GetY()-a.GetY());
    return true;
  }
};

int main(){
  auto* laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("test"));
  laser->SetMinimumRange(.1);laser->SetMaximumRange(12.);laser->SetRangeThreshold(11.9);
  laser->SetMinimumAngle(-3.141592653589793);laser->SetMaximumAngle(3.141592653589793);
  laser->SetAngularResolution(2*3.141592653589793/359);
  karto::SensorManager::GetInstance()->RegisterSensor(laser);
  karto::RangeReadingsVector ranges(360,std::numeric_limits<double>::quiet_NaN());
  ranges[0]=3.;ranges[1]=std::nextafter(12.f,0.f);ranges[2]=INFINITY;ranges[3]=-INFINITY;ranges[4]=12.;
  karto::LocalizedRangeScan scan(karto::Name("test"),ranges);
  scan.SetCorrectedPose(karto::Pose2(0.,0.,0.));
  InspectGrid grid;grid.AddScan(&scan);
  assert(grid.traces==2);assert(grid.occupied_endpoints==1);
  assert(std::abs(grid.clearing_length-11.9)<1.e-8);
  std::cout<<"PASS: installed Karto clears normalized no-return to 11.9 m without an occupied endpoint; finite hit preserved; invalid/max ranges ignored.\n";
}
