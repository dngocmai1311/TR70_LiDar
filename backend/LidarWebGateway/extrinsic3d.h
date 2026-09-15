#pragma once
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

// Metres and radians internally; configuration and UI angles are degrees.
struct Extrinsic3D {
    float x = 0, y = 0, z = 0;
    float roll = 0, pitch = 0, yaw = 0;
};
struct Point3D { float x, y, z; };
struct Transform3D {
    float r[9];
    Point3D t;
    explicit Transform3D(const Extrinsic3D& e) : t{e.x,e.y,e.z} {
        const float cr=std::cos(e.roll), sr=std::sin(e.roll);
        const float cp=std::cos(e.pitch), sp=std::sin(e.pitch);
        const float cy=std::cos(e.yaw), sy=std::sin(e.yaw);
        const float m[9]={cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,
            sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};
        for(int i=0;i<9;++i) r[i]=m[i];
    }
    Point3D Apply(float x,float y,float z=0) const {
        return {r[0]*x+r[1]*y+r[2]*z+t.x,
            r[3]*x+r[4]*y+r[5]*z+t.y,r[6]*x+r[7]*y+r[8]*z+t.z};
    }
};
inline Point3D TransformPoint(float x,float y,float z,const Extrinsic3D& e) {
    return Transform3D(e).Apply(x,y,z);
}
inline bool ParseExtrinsic(const std::string& line, Extrinsic3D& out) {
    std::istringstream in(line.substr(0,line.find('#')));
    std::vector<float> v; float a;
    while(in>>a) { if(!std::isfinite(a)) return false; v.push_back(a); }
    if(!in.eof() || (v.size()!=3 && v.size()!=6)) return false;
    constexpr float radians=3.14159265358979f/180.f;
    Extrinsic3D e; e.x=v[0]; e.y=v[1];
    if(v.size()==3) e.yaw=v[2]*radians;
    else { e.z=v[2]; e.roll=v[3]*radians; e.pitch=v[4]*radians; e.yaw=v[5]*radians; }
    out=e; return true;
}
