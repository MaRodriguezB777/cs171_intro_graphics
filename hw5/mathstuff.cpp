#include "mathstuff.hpp"

Vector3f calc_normal(HEF *f) {
    HEV *hev1 = f->edge->vertex;
    HEV *hev2 = f->edge->next->vertex;
    HEV *hev3 = f->edge->next->next->vertex;

    Vector3f v1(hev1->x, hev1->y, hev1->z);
    Vector3f v2(hev2->x, hev2->y, hev2->z);
    Vector3f v3(hev3->x, hev3->y, hev3->z);

    Vector3f face_normal = (v2 - v1).cross(v3 - v1);

    return face_normal;
}

void index_vertices(std::vector<HEV *> *vertices)
{
    for (int i = 1; i < vertices->size(); i++) {
        vertices->at(i)->index = i;
    }
}

double norm(HEV *v1, HEV *v2) {
    double diff_x = v1->x - v2->x;
    double diff_y = v1->y - v2->y;
    double diff_z = v1->z - v2->z;

    return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

double calc_area(HE *he) {

    double tot_area_sum = 0.0;
    HEV *v = he->vertex;

    do
    {
        // compute the normal of the plane of the face
        Vector3f face_normal = calc_normal(he->face);
        // compute the area of the triangular face
        double face_area = 0.5 * face_normal.norm();
        tot_area_sum += face_area;

        he = he->flip->next;
    } while(he != v->out);

    return tot_area_sum;
}

double calc_cos_rad(HEV *mid, HEV *end1, HEV *end2)
{
    Eigen::Vector3d vec1(end1->x - mid->x, end1->y - mid->y, end1->z - mid->z);
    Eigen::Vector3d vec2(end2->x - mid->x, end2->y - mid->y, end2->z - mid->z);

    return vec1.dot(vec2) / (vec1.norm() * vec2.norm());
}

double calc_sin_rad(HEV *mid, HEV *end1, HEV *end2)
{
    Eigen::Vector3d vec1(end1->x - mid->x, end1->y - mid->y, end1->z - mid->z);
    Eigen::Vector3d vec2(end2->x - mid->x, end2->y - mid->y, end2->z - mid->z);

    return vec1.cross(vec2).norm() / (vec1.norm() * vec2.norm());
}

// function to construct our B operator in matrix form
Eigen::SparseMatrix<double> build_F_operator( std::vector<HEV *> *vertices, double h )
{
    std::cout << "h = " << h << std::endl;
    index_vertices( vertices ); // assign each vertex an index

    // recall that due to 1-indexing of obj files, index 0 of our list doesnt actually contain a vertex
    int num_vertices = vertices->size() - 1;

    // initialize a sparse matrix to represent our B operator
    Eigen::SparseMatrix<double> B( num_vertices, num_vertices );

    // reserve room for 7 non-zeros per row of B
    B.reserve( Eigen::VectorXi::Constant( num_vertices, 7 ) );

    for( int i = 1; i < vertices->size(); ++i )
    {
        HE *he = vertices->at(i)->out;
        double tot_area_sum = calc_area(he);

        // Diagonal
        B.insert(i - 1, i - 1) = 1.0;

        // Degenerate
        if (tot_area_sum < 1e-6) {
            continue;
        }

        double running_cot_sum = 0;

        do // iterate over all vertices adjacent to v_i
        {
            int j = he->next->vertex->index; // get index of adjacent vertex to v_i
            HEV *alpha_v = he->next->next->vertex; // might be next->flip not next->next
            HEV *beta_v = he->flip->next->next->vertex;
            HEV *j_v = he->next->vertex;
            HEV *curr = he->vertex;

            // alpha is equal to angle between (j_v - alpha_v) and (curr - alpha_v)
            // beta is equal to angle between (j_v - beta_v) and (curr - beta_v)
            double cot_alpha = calc_cos_rad(alpha_v, j_v, curr) / calc_sin_rad(alpha_v, j_v, curr);
            double cot_beta = calc_cos_rad(beta_v, j_v, curr) / calc_sin_rad(beta_v, j_v, curr);

            // call function to compute edge length
            double edge_length = -h * (cot_alpha + cot_beta) / (2 * tot_area_sum);
            // fill the j-th slot of row i of our B matrix with appropriate value
            B.insert( i-1, j-1 ) = edge_length;

            running_cot_sum += edge_length;
            he = he->flip->next;
        } while( he != vertices->at(i)->out );

        B.coeffRef(i - 1, i - 1) -= running_cot_sum;
    }

    B.makeCompressed(); // optional; tells Eigen to more efficiently store our sparse matrix
    return B;
}

void solve( std::vector<HEV *> *vertices, double h )
{
    // get our matrix representation of B
    Eigen::SparseMatrix<double> B = build_F_operator( vertices, h );

    // initialize Eigens sparse solver
    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > solver;

    // the following two lines essentially tailor our solver to our operator B
    solver.analyzePattern( B );
    solver.factorize( B );

    int num_vertices = vertices->size() - 1;

    // initialize our vector representation of rho
    Eigen::VectorXd rho_x( num_vertices );
    Eigen::VectorXd rho_y( num_vertices );
    Eigen::VectorXd rho_z( num_vertices );
    for( int i = 1; i < vertices->size(); ++i ) {
        rho_x(i - 1) = vertices->at(i)->x;
        rho_y(i - 1) = vertices->at(i)->y;
        rho_z(i - 1) = vertices->at(i)->z;
    }

    // have Eigen solve for our phi_vectors (x, y, z)
    Eigen::VectorXd phi_xh( num_vertices );
    Eigen::VectorXd phi_yh( num_vertices );
    Eigen::VectorXd phi_zh( num_vertices );

    phi_xh = solver.solve( rho_x );
    phi_yh = solver.solve( rho_y );
    phi_zh = solver.solve( rho_z );

    for (int i = 1; i < vertices->size(); ++i) {
        HEV *v = vertices->at(i);
        v->x = phi_xh(i - 1);
        v->y = phi_yh(i - 1);
        v->z = phi_zh(i - 1);
    }
}
