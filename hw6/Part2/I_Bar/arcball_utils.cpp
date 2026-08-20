#include "arcball_utils.hpp"

MyQuaternion MyQuaternion::operator*(MyQuaternion& q) {
    return MyQuaternion(
        s*q.s - v.dot(q.v),
        s*q.v + q.s*v + v.cross(q.v)
    );
}

MyQuaternion MyQuaternion::Identity() {
    return MyQuaternion(1.0, 0.0, 0.0, 0.0);
}

// `s` given in degrees
MyQuaternion::MyQuaternion(float *rot_info) {
    float s_rad = rot_info[3] * M_PI / 180.0;

    float sinHalfAngle = sin(s_rad / 2);
    float cosHalfAngle = cos(s_rad / 2);

    float norm = sqrt(rot_info[0] * rot_info[0] + rot_info[1] * rot_info[1] + rot_info[2] * rot_info[2]);
    
    this->s = cosHalfAngle;
    this->v << rot_info[0] * sinHalfAngle / norm,
               rot_info[1] * sinHalfAngle / norm,
               rot_info[2] * sinHalfAngle / norm;
    
    this->normalize();
}

void MyQuaternion::normalize() {
    double norm = sqrt(s*s + v.dot(v));
    s /= norm;
    v /= norm;
}

Vector3d map_to_sphere(int x, int y, int xres, int yres) {
    // convert x and y to ndc
    double ndc_x = 1.0 * ((2.0 * x) / xres - 1.0);
    double ndc_y = -1 * ((2.0 * y) / yres - 1.0); // inverted yaxis to match pixels
    double dist2 = ndc_x * ndc_x + ndc_y * ndc_y;

    Vector3d sphere_pos;
    if (dist2 > 1.0) {
        double dist = sqrt(dist2);
        sphere_pos = Vector3d(ndc_x / dist, ndc_y / dist, 0.0);
    } else {
        sphere_pos = Vector3d(ndc_x, ndc_y, sqrt(1.0 - dist2));
    }
    return sphere_pos;
}
