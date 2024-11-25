#include "Eigen/Dense"
#include "Eigen/Sparse"

#include "opengl_utils.hpp"
#include "halfedge.h"

Eigen::SparseMatrix<double> build_F_operator(std::vector<HEV *> *vertices, double h);
void solve(std::vector<HEV *> *vertices, double h);
Vector3f calc_normal(HEF *f);
