#ifndef ARCBALL_UTILS_HPP
#define ARCBALL_UTILS_HPP
#include "Eigen/Dense"

using namespace Eigen;

struct MyQuaternion {
    float s;
    Vector3f v;

    MyQuaternion() {}
    MyQuaternion(float s, float x, float y, float z) : s(s), v(x, y, z) {}
    MyQuaternion(float s, Vector3f v) : s(s), v(v) {}
    MyQuaternion(float *rot_info);
    MyQuaternion operator*(MyQuaternion& q);
    static MyQuaternion Identity();
    void normalize();
};

Vector3d map_to_sphere(int x, int y, int xres, int yres);

#endif