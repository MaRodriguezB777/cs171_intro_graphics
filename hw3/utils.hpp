#ifndef UTILS_HPP
#define UTILS_HPP

#include "Eigen/Dense"

using namespace Eigen;

enum ShadingType {
    GOURAUD,
    PHONG
};

enum TransformType
{
    TT_TRANSLATE,
    TT_ROTATE,
    TT_SCALE
};


struct Face {
    int v1, v2, v3; // 0-indexed
    int n1, n2, n3; // 0-indexed
    Face(int v1, int v2, int v3, int n1, int n2, int n3);
};

struct Color {
    float r, g, b;
    Color(float r, float g, float b);
    Color() {}
};

struct Light {
    Vector3f position;
    Color c;
    float k; // attenuation param

    Light(float x, float y, float z, float r, float g, float b, float k);
};

struct Point {
    Vector4f v;
    Color c;

    Point(Vector4f position);
    Point() {}
};

struct TransformInfo
{
    TransformType type;
    /* For each array below,
     * Index 0 has the x-component
     * Index 1 has the y-component
     * Index 2 has the z-component
     */
    float values[3];

    /* Angle in degrees.
     */
    float rotation_angle;
};

struct Shape {
    std::string name;
    std::string filename;
    Color ambient;
    Color diffuse;
    Color specular;
    float shininess;
    std::vector<Face> faces;
    std::vector<Point> points; // not transformed
    std::vector<Vector3f> normals; // not transformed
    std::vector<TransformInfo> transforms;

    Shape(  std::string& name,
            std::string& filename,
            std::vector<Face>& faces,
            std::vector<Vector4f>& world_vertices,
            std::vector<Vector3f>& normals,
            std::vector<TransformInfo>& transforms,
            Color& ambient,
            Color& diffuse,
            Color& specular,
            float shininess);

    Shape() {}
};

struct Object {
    std::string name;
    std::string filename;
    std::vector<Vector4f> vertices; // 0-indexed
    std::vector<Vector3f> surface_normals; // not normalized
    std::vector<Face> faces;

    Object(std::string name, std::string filename);
};

struct Camera {
    Vector3f position;
    Vector3f rot_axis;
    float rot_angle;
    float n; // magnitude of negative z-coordinate at the near plane
    float f; // magnitude of the negative z-coordinate at the far plane
    float l;
    float r;
    float t;
    float b;

    Camera(Vector3f& position, 
        Vector3f& rot_axis, 
        float rot_angle,
        float near, 
        float far, 
        float left, 
        float right, 
        float top, 
        float bottom);

    Camera() {}
};

struct Scene {
    Camera camera;
    Matrix4f space_matrix; // World to Camera Coordinates
    Matrix4f pers_matrix; // Camera to Normalized Device Coordinates
    std::vector<Shape> shapes;
    std::vector<Light> lights;

    Scene(Camera& camera, std::vector<Shape>& shapes, std::vector<Light>& lights);
    Scene() {}
    void calc_space_transform_matrix();
    void calc_pers_matrix();
};

void calc_translation_matrix(Vector3f& v, Matrix4f& T);
void calc_rotation_matrix(Vector3f& u, float angle, Matrix4f& R);
void calc_scaling_matrix(Vector3f& v, Matrix4f& S);
void draw_line(int x0, int y0, int x1, int y1, std::vector<std::vector<bool> >& img);
void color_pixel(int x, int y, std::vector<std::vector<bool> >& img);
Camera read_camera_section(std::ifstream& file);
void read_objects_section(std::ifstream& file, std::unordered_map<std::string, Object>& objects);
Shape create_shape(std::string name, std::string filename,std::ifstream& file, Object& obj);
Vector4f transform_to_ndc(Matrix4f& space_matrix, Matrix4f& pers_matrix, Vector4f v);
void calc_lighting_model(
    Color ambient_c,
    Color diffuse_c,
    Color specular_c,
    float shininess_phong,
    Point& p,
    Vector3f& n,
    std::vector<Light>& lights,
    Vector3f& cam_position);
void rasterize_triangle_gouraud(
    Point& p1,
    Point& p2,
    Point& p3,
    int xres,
    int yres,
    std::vector<std::vector<Color> >& img,
    std::vector<std::vector<float> >& depth_buffer);
void rasterize_triangle_phong(
    Point& p1,
    Point& p2,
    Point& p3,
    Vector3f& n1,
    Vector3f& n2,
    Vector3f& n3,
    Shape& shape,
    Matrix4f& pers_matrix,
    Matrix4f& space_matrix,
    std::vector<Light>& lights,
    Vector3f& cam_position,
    int xres,
    int yres,
    std::vector<std::vector<Color> >& img,
    std::vector<std::vector<float> >& depth_buffer);

void fill_grid(Scene scene, int xres, int yres, std::vector<std::vector<Color> >& img, ShadingType shading_type);
void print_ppm(std::vector<std::vector<Color> >& img, int xres, int yres, int color_size);

#endif // UTILS_HPP