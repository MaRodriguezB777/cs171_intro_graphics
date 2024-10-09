#include <Eigen/Dense>
#include <iostream>
#include <fstream>

using namespace Eigen;

void calc_translation_matrix(Vector3d& v, Matrix4d& T) {
    T << 1, 0, 0, v(0),
         0, 1, 0, v(1),
         0, 0, 1, v(2),
         0, 0, 0, 1;
}

void calc_rotation_matrix(Vector3d& u, double angle, Matrix4d& R) {
    double ux2 = u(0) * u(0);
    double uy2 = u(1) * u(1);
    double uz2 = u(2) * u(2);
    double uxy = u(0) * u(1);
    double uxz = u(0) * u(2);
    double uyz = u(1) * u(2);
    double ux = u(0);
    double uy = u(1);
    double uz = u(2);

    R << ux2 + (1 - ux2) * cos(angle), uxy * (1 - cos(angle)) - uz * sin(angle), uxz * (1 - cos(angle)) + uy * sin(angle), 0,
         uxy * (1 - cos(angle)) + uz * sin(angle), uy2 + (1 - uy2) * cos(angle), uyz * (1 - cos(angle)) - ux * sin(angle), 0,
         uxz * (1 - cos(angle)) - uy * sin(angle), uyz * (1 - cos(angle)) + ux * sin(angle), uz2 + (1 - uz2) * cos(angle), 0,
         0, 0, 0, 1;
}

void calc_scaling_matrix(Vector3d& v, Matrix4d& S) {
    S << v(0), 0, 0, 0,
         0, v(1), 0, 0,
         0, 0, v(2), 0,
         0, 0, 0, 1;
}

int main(int argc, char** argv) {
    /*
    File contains a list of translation, rotation, and scaling vectors
    File format:
        t tx ty tz
        - t indicates a translation vector
        r rx ry rz angle_in_radians
        - r indicates a rotation vector
        s sx sy sz
        - s indicates a scaling vector

    Want to calculate the corresponding translation, rotation, and scaling
    matrices, multiply them together, and then get the inverse of the
    resulting matrix.
    */
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " file.txt" << std::endl;
        return 1;
    }
    std::string filename = argv[1];

   std::ifstream file(filename);
   std::string line;
   Matrix4d product = Matrix4d::Identity();

   while (std::getline(file, line)) {
       std::istringstream iss(line);
       std::string type;
       iss >> type;

       Matrix4d M;
       if (type == "t") {
           Vector3d v;
           double tx, ty, tz;
           iss >> tx >> ty >> tz;
           v << tx, ty, tz;
           calc_translation_matrix(v, M);
       } else if (type == "r") {
           Vector3d v;
           Matrix4d R;
           double rx, ry, rz, angle;
           iss >> rx >> ry >> rz >> angle;
           v << rx, ry, rz;
           calc_rotation_matrix(v, angle, M);
       } else if (type == "s") {
           Vector3d v;
           Matrix4d S;
           double sx, sy, sz;
           iss >> sx >> sy >> sz;
           v << sx, sy, sz;
           calc_scaling_matrix(v, M);
       }

       product = M * product;
   }

   file.close();
   Matrix4d inv_m = product.inverse();
   std::cout << inv_m << std::endl;
    return 0;
}
