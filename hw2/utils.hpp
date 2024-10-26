#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>

using namespace Eigen;

enum ShadingType {
    GOURAUD,
    PHONG
};

struct Face {
    int v1, v2, v3; // 0-indexed
    int n1, n2, n3; // 0-indexed
    Face(int v1, int v2, int v3, int n1, int n2, int n3);
};

struct Color {
    double r, g, b;
    Color(double r, double g, double b);
    Color() {}
};

struct Light {
    Vector3d position;
    Color c;
    double k; // attenuation param

    Light(double x, double y, double z, double r, double g, double b, double k);
};

struct Point {
    Vector4d v;
    Color c;

    Point(Vector4d position);
    Point() {}
};

struct Shape {
    std::string name;
    std::string filename;
    Color ambient;
    Color diffuse;
    Color specular;
    double shininess;
    std::vector<Face> faces;
    std::vector<Point> points;
    std::vector<Vector3d> normals;

    Shape(  std::string& name,
            std::string& filename,
            std::vector<Face>& faces,
            std::vector<Vector4d>& world_vertices,
            std::vector<Vector3d>& normals,
            Color& ambient,
            Color& diffuse,
            Color& specular,
            double shininess);
};

struct Object {
    std::string name;
    std::string filename;
    std::vector<Vector4d> vertices; // 0-indexed
    std::vector<Vector3d> surface_normals; // not normalized
    std::vector<Face> faces;
    // std::vector<Matrix4d> transform_matrices;
    // std::vector<Shape> shapes; // This will replace the transformed_vertices
    // std::vector<std::vector<Vector4d> > transformed_vertices;
    // std::vector<std::vector<Vector4d*> > cartesian_ndc;
    // std::vector<std::vector<Vector2d*> > screen_coords;

    Object(std::string name, std::string filename);
    // void apply_geometric_transforms();
    std::vector<std::vector<Vector4d*>> get_cartesian_ndc(Matrix4d& space_matrix, Matrix4d& pers_matrix);
    void apply_transforms_and_cartesian_ndc(Matrix4d& space_matrix, Matrix4d& pers_matrix);
    void apply_screen_mapping(int xres, int yres);
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

    Camera() {}
};

struct Scene {
    Camera camera;
    Matrix4d space_matrix; // World to Camera Coordinates
    Matrix4d pers_matrix; // Camera to Normalized Device Coordinates
    std::vector<Shape> shapes;
    std::vector<Light> lights;

    Scene(Camera& camera, std::vector<Shape>& shapes, std::vector<Light>& lights);
    void calc_space_transform_matrix();
    void calc_pers_matrix();
};

void calc_translation_matrix(Vector3d& v, Matrix4d& T);
void calc_rotation_matrix(Vector3d& u, double angle, Matrix4d& R);
void calc_scaling_matrix(Vector3d& v, Matrix4d& S);
void draw_line(int x0, int y0, int x1, int y1, std::vector<std::vector<bool>>& img);
void color_pixel(int x, int y, std::vector<std::vector<bool>>& img);
Camera read_camera_section(std::ifstream& file);
void read_objects_section(std::ifstream& file, std::unordered_map<std::string, Object>& objects);
Shape create_shape(std::string name, std::string filename,std::ifstream& file, Object& obj);
Vector4d transform_to_ndc(Matrix4d& space_matrix, Matrix4d& pers_matrix, Vector4d v);
void calc_lighting_model(
    Color ambient_c,
    Color diffuse_c,
    Color specular_c,
    double shininess_phong,
    Point& p,
    Vector3d& n,
    std::vector<Light>& lights,
    Vector3d& cam_position);
void rasterize_triangle_gouraud(
    Point& p1,
    Point& p2,
    Point& p3,
    int xres,
    int yres,
    std::vector<std::vector<Color>>& img,
    std::vector<std::vector<double>>& depth_buffer);
void rasterize_triangle_phong(
    Point& p1,
    Point& p2,
    Point& p3,
    Vector3d& n1,
    Vector3d& n2,
    Vector3d& n3,
    Shape& shape,
    Matrix4d& pers_matrix,
    Matrix4d& space_matrix,
    std::vector<Light>& lights,
    Vector3d& cam_position,
    int xres,
    int yres,
    std::vector<std::vector<Color>>& img,
    std::vector<std::vector<double>>& depth_buffer);

void fill_grid(Scene scene, int xres, int yres, std::vector<std::vector<Color>>& img, ShadingType shading_type);
void print_ppm(std::vector<std::vector<Color>>& img, int xres, int yres, int color_size);
#endif // UTILS_HPP