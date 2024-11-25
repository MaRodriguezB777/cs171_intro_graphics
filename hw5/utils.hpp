#ifndef UTILS_HPP
#define UTILS_HPP

#include "Eigen/Dense"
#include "halfedge.h"

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
    std::vector<HEV*> *hevs;
    std::vector<HEF*> *hefs;
    std::vector<TransformInfo> transforms;

    Shape(  std::string& name,
            std::string& filename,
            std::vector<HEF*> *hefs,
            std::vector<HEV*> *hevs,
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
    Mesh_Data *mesh_data; // 1-indexed
    std::vector<HEV*> *hevs; // 1-indexed
    std::vector<HEF*> *hefs;
    
    Object(std::string name, std::string filename, std::string directory);
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
    Camera *camera;
    std::vector<Shape *> *shapes;
    std::vector<Light *> *lights;
    int xres, yres;

    Scene(Camera *camera, std::vector<Shape *> *shapes, std::vector<Light *> *lights);
    Scene() {}
};

void calc_translation_matrix(Vector3f& v, Matrix4f& T);
void calc_rotation_matrix(Vector3f& u, float angle, Matrix4f& R);
void calc_scaling_matrix(Vector3f& v, Matrix4f& S);
void draw_line(int x0, int y0, int x1, int y1, std::vector<std::vector<bool> >& img);
void color_pixel(int x, int y, std::vector<std::vector<bool> >& img);
Camera read_camera_section(std::ifstream& file);
void read_objects_section(std::ifstream& file, std::unordered_map<std::string, Object *> *objects, std::string directory);
Shape create_shape(std::string name, std::string filename, std::ifstream& file, Object& obj);
Vector4f transform_to_ndc(Matrix4f& space_matrix, Matrix4f& pers_matrix, Vector4f v);

#endif // UTILS_HPP