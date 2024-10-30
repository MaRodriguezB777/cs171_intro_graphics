#ifndef OPENGL_UTILS_HPP
#define OPENGL_UTILS_HPP

#include "utils.hpp"

void set_scene(Scene& new_scene);
void init();
void reshape(int width, int height);
void display();
void init_lights();
void set_lights();
void draw_objects();
void create_objects_from_scene();
void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);

struct Triple {
    float x;
    float y;
    float z;

    Triple(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct OpenGL_Shape {
    std::vector<Triple> vertex_buffer;
    std::vector<Triple> normal_buffer;

    std::vector<TransformInfo> transforms;

    float ambient[3];
    float diffuse[3];
    float specular[3];
    float shininess;

};

#endif