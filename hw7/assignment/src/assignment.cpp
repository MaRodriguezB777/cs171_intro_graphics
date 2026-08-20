#include "object.h"
#include "scene.h"

#include <iostream>

#include "image.h"

using namespace Eigen;
using namespace std;

const int MAX_ITERS = 10000;
const int XRES = 500;
const int YRES = 500;

/**
 * IOTest Code
 */

bool Superquadric::IOTest(const Vector3d &point) {
    /**
     * PART 1
     * TODO: Implement the IO Test function for a superquadric. Remember to
     *       apply any transformations to the superquadric before testing
     *       against the IO function.
     */

    // Convert the world-space point to a homogeneous coordinate
    Eigen::Vector4d p_homog(point(0), point(1), point(2), 1.0);

    // Apply the inverse of each transform in reverse order (rbegin to rend)
    // to go from world space to the superquadric's object space.
    Eigen::Matrix4d inverse = Matrix4d::Identity();
    for (int i = 0; i < this->transforms.size(); i++) {
        inverse *= this->transforms[i]->GetMatrix().inverse();
    }
    p_homog = inverse * p_homog;

    // Extract the local coordinates and use absolute values for the equation
    double X = std::abs(p_homog(0));
    double Y = std::abs(p_homog(1));
    double Z = std::abs(p_homog(2));

    // Compute the superquadric IO function value.
    // val = pow((X^(2/exp0) + Y^(2/exp0)), exp0/exp1) + (Z^(2/exp1)) - 1
    double temp = std::pow(X * X, 1.0 / this->exp0) + std::pow(Y * Y, 1.0 / this->exp0);
    double val = -1.0 + std::pow(Z * Z, 1.0 / this->exp1) + std::pow(temp, this->exp0 / this->exp1);

    // If val < 0, the point is inside the superquadric.
    return (val < 0.0);
}

bool Assembly::IOTest(const Vector3d &point) {
    /**
     * PART 1
     * TODO: Implement the IO Test function for an assembly (recursively call
     *       IOTest on the children). Make sure to apply any transformations
     *       to the assembly before calling IOTest on the children.
     */

    // Convert the world-space point to a homogeneous coordinate
    Eigen::Vector4d p_homog(point(0), point(1), point(2), 1.0);

    // Apply the inverse of each transform in reverse order
    // to get the point in the assembly's local coordinate system.
    Eigen::Matrix4d inverse = Matrix4d::Identity();
    for (int i = 0; i < this->transforms.size(); i++) {
        inverse *= this->transforms[i]->GetMatrix().inverse();
    }
    p_homog = inverse * p_homog;

    // Now p is in assembly space
    Eigen::Vector3d p = p_homog.head<3>();

    // Recursively check children. If any child returns true, we are inside the assembly.
    for (auto &child : this->children) {
        bool inside = child->IOTest(p);
        
        if (inside) {
            return true;
        }
    }

    return false;
}

/**
 * Closest Intersection Code
 */

pair<double, Intersection> Superquadric::ClosestIntersection(const Ray &ray) {
    /**
     * PART 1
     * TODO: Implement a ray-superquadric intersection using Newton's method.
     *       Make sure to apply any transformations to the superquadric before
     *       performing Newton's method.
     */
    pair<double, Intersection> closest = make_pair(INFINITY, Intersection());

    Ray body_ray = ray.Transformed(getInverseTransformMatrix());

    
    return closest;
}

pair<double, Intersection> Assembly::ClosestIntersection(const Ray &ray) {
    /**
     * PART 1
     * TODO: Implement a ray-assembly intersection by recursively finding
     *       intersection with the assembly's children. Make sure to apply any
     *       transformations to the assembly before calling ClosestIntersection
     *       on the children.
     */
    pair<double, Intersection> closest = make_pair(INFINITY, Intersection());
    return closest;
}

/**
 * Raytracing Code
 */

void Scene::Raytrace() {
    Image img = Image(XRES, YRES);

    for (int i = 0; i < XRES; i++) {
        for (int j = 0; j < YRES; j++) {
            /**
             * PART 2
             * TODO: Implement raytracing using the code from the first part
             *       of the assignment. Set the correct color for each pixel
             *       here.
             */
            img.SetPixel(i, j, Vector3f::Ones());
        }
    }

    // Outputs the image.
    if (!img.SaveImage("rt.png")) {
        cerr << "Error: couldn't save PNG image" << std::endl;
    } else {
        cout << "Done!\n";
    }
}
