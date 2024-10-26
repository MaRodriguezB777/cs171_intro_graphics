#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>

using namespace Eigen;

struct Face {
    int v1, v2, v3;
};

struct Object {
    std::string name;
    std::string filename;
    std::vector<Vector4d> vertices; // 0-indexed
    std::vector<Face> faces; // 1-indexed
    std::vector<Matrix4d> transform_matrices;
    std::vector<std::vector<Vector4d> > transformed_vertices;
    std::vector<std::vector<Vector4d*> > cartesian_ndc;
    std::vector<std::vector<Vector2d*> > screen_coords;


    Object(std::string name, std::string filename);
    void apply_geometric_transforms();
    std::vector<std::vector<Vector4d*>> get_cartesian_ndc(Matrix4d& space_matrix, Matrix4d& pers_matrix);
    void apply_transforms_and_cartesian_ndc(Matrix4d& space_matrix, Matrix4d& pers_matrix);
    void apply_screen_mapping(int xres, int yres);
    // std::vector<std::vector<Vector4d>> draw_faces();
};

struct Camera {
    Vector3d position;
    Vector3d rot_axis;
    double rot_angle;
    double n; // magnitude of negative z-coordinate at the near plane
    double f; // magnitude of the negative z-coordinate at the far plane
    double l;
    double r;
    double t;
    double b;

    Camera(Vector3d& position, 
        Vector3d& rot_axis, 
        double rot_angle,
        double near, 
        double far, 
        double left, 
        double right, 
        double top, 
        double bottom);
};

struct Scene {
    Camera *camera;
    Matrix4d space_matrix; // World to Camera Coordinates
    Matrix4d pers_matrix; // Camera to Normalized Device Coordinates
    std::unordered_map<std::string, Object> objects_map;

    Scene(Camera *camera, std::unordered_map<std::string, Object>& obj_transforms);
    void calc_space_transform_matrix();
    void calc_pers_matrix();
};

void calc_translation_matrix(Vector3d& v, Matrix4d& T);
void calc_rotation_matrix(Vector3d& u, double angle, Matrix4d& R);
void calc_scaling_matrix(Vector3d& v, Matrix4d& S);
void draw_line(int x0, int y0, int x1, int y1, std::vector<std::vector<bool>>& img);
void color_pixel(int x, int y, std::vector<std::vector<bool>>& img);

#endif // UTILS_HPP