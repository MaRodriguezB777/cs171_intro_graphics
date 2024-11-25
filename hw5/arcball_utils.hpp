#ifndef ARCBALL_UTILS_HPP
#define ARCBALL_UTILS_HPP
#include "Eigen/Dense"

using namespace Eigen;

struct MyQuaternion {
    double s;
    Vector3d v;

    MyQuaternion() {}
    MyQuaternion(double s, double x, double y, double z) : s(s), v(x, y, z) {}
    MyQuaternion(double s, Vector3d v) : s(s), v(v) {}
    MyQuaternion operator*(MyQuaternion& q);
    static MyQuaternion Identity();
    void normalize();
};

Vector3d map_to_sphere(int x, int y, int xres, int yres);

#endif