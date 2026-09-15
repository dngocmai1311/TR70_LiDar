#include "../backend/LidarWebGateway/extrinsic3d.h"
#include <cassert>
#include <iostream>
bool near(float a,float b){return std::abs(a-b)<1e-5f;}
int main(){
    Extrinsic3D e;
    assert(ParseExtrinsic("5 6 90",e)); assert(e.z==0&&e.roll==0&&e.pitch==0);
    auto p=TransformPoint(3,0,0,e);assert(near(p.x,5)&&near(p.y,9)&&near(p.z,0));
    for(float z:{.4f,.7f,1.f,1.3f}){
        e={0,0,z,0,0,0};
        for(float y:{-2.f,0.f,2.f}){p=TransformPoint(3,y,0,e);assert(near(p.x,3)&&near(p.y,y)&&near(p.z,z));}
    }
    assert(ParseExtrinsic("0 0 1 0 30 0",e));p=TransformPoint(3,0,0,e);assert(near(p.z,-.5f));
    assert(ParseExtrinsic("0 0 1 90 0 0",e));p=TransformPoint(0,2,0,e);assert(near(p.y,0)&&near(p.z,3));
    assert(ParseExtrinsic("1 2 3 20 30 40 # degrees",e));
    p=TransformPoint(3,4,5,e);assert(near(p.x,3.997003f)&&near(p.y,7.189133f)&&near(p.z,6.753781f));
    assert(!ParseExtrinsic("1 2",e));assert(!ParseExtrinsic("1 2 3 4",e));assert(!ParseExtrinsic("1 2 nan",e));assert(!ParseExtrinsic("1 2 3 junk",e));
    std::cout<<"PASS: extrinsic legacy/new formats, four wall layers, yaw/pitch/roll and compound rotation\n";
}
