#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <cstdlib>
#include <vector>
#include <unordered_map>
#include <GLUT/glut.h>

#include "utils.hpp"
#include "opengl_utils.hpp"

bool PRINT_MAIN = false;

void print_parsed_scene(Scene& scene, std::unordered_map<std::string, Object> objects) {
    // make sure camera is correct by printing all field in Camera object
    Camera camera = scene.camera;
    std::cout << "Camera position: " << camera.position.transpose() << std::endl;
    std::cout << "Camera rotation axis: " << camera.rot_axis.transpose() << std::endl;
    std::cout << "Camera rotation angle: " << camera.rot_angle << std::endl;
    std::cout << "Camera near: " << camera.n << std::endl;
    std::cout << "Camera far: " << camera.f << std::endl;
    std::cout << "Camera left: " << camera.l << std::endl;
    std::cout << "Camera right: " << camera.r << std::endl;
    std::cout << "Camera top: " << camera.t << std::endl;
    std::cout << "Camera bottom: " << camera.b << std::endl;

    // make sure lights are correct
    for (auto& light : scene.lights) {
        std::cout << "Light position: " << light.position.transpose() << std::endl;
        std::cout << "Light color: " << light.c.r << " " << light.c.g << " " << light.c.b << std::endl;
        std::cout << "Light attenuation: " << light.k << std::endl;
    }

    for (auto& obj : objects) {
        std::cout << "Object name: " << obj.first << std::endl;
        std::cout << "Object filename: " << obj.second.filename << std::endl;
        std::cout << "Object surface normals: " << std::endl;
        for (auto& normal : obj.second.surface_normals) {
            std::cout << normal.transpose() << std::endl;
        }
        std::cout << "Object vertices: " << std::endl;
        for (auto& v : obj.second.vertices) {
            std::cout << v.transpose() << std::endl;
        }
        std::cout << "Object faces: " << std::endl;
        for (auto& face : obj.second.faces) {
            std::cout << face.v1 + 1 << "//" << face.n1 + 1 << " " << face.v2 + 1 << "//" << face.n2 + 1 << " " << face.v3 + 1 << "//" << face.n3 + 1 << std::endl;
        }
    }

    for (auto& shape : scene.shapes) {
        std::cout << "Shape name: " << shape.name << std::endl;
        std::cout << "Shape filename: " << shape.filename << std::endl;
        std::cout << "Shape ambient: " << shape.ambient.r << " " << shape.ambient.g << " " << shape.ambient.b << std::endl;
        std::cout << "Shape diffuse: " << shape.diffuse.r << " " << shape.diffuse.g << " " << shape.diffuse.b << std::endl;
        std::cout << "Shape specular: " << shape.specular.r << " " << shape.specular.g << " " << shape.specular.b << std::endl;
        std::cout << "Shape shininess: " << shape.shininess << std::endl;

        std::cout << "Shape vertices: " << std::endl;
        for (auto& p : shape.points) {
            std::cout << p.v.transpose() << std::endl;
        }
        std::cout << "Shape normals: " << std::endl;
        for (auto& v : shape.normals) {
            std::cout << v.transpose() << std::endl;
        }
        std::cout << "NDC vertices: " << std::endl;
        for (auto& p : shape.points) {
            Vector4f v = transform_to_ndc(scene.space_matrix, scene.pers_matrix, p.v);
            std::cout << v.transpose() << std::endl;
        }
    }

    std::cout << "Space matrix: " << std::endl;
    std::cout << scene.space_matrix << std::endl;

    std::cout << "Perspective matrix: " << std::endl;
    std::cout << scene.pers_matrix << std::endl;
}

Scene read_scene_file(std::string filename) {
    Camera *camera_ptr = nullptr;
    std::unordered_map<std::string, Object> objects;
    std::vector<Shape> shapes;
    std::vector<Light> lights;

    std::ifstream file(filename);
    std::string line;
    while(std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "camera:") {
            camera_ptr = new Camera(read_camera_section(file));
        } else if (type == "light") {
            float x, y, z;
            iss >> x >> y >> z;
            std::string comma;
            iss >> comma;
            float r, g, b;
            iss >> r >> g >> b;
            iss >> comma;
            float k;
            iss >> k;
            lights.push_back(Light(x, y, z, r, g, b, k));
        } else if (type == "objects:") {
            read_objects_section(file, objects);
        } else if (objects.find(type) != objects.end()) {
            Object obj = objects.find(type)->second;
            shapes.push_back(create_shape(obj.name, obj.filename, file, objects.at(type)));
        } else {
            std::cerr << "Unknown type in scene file: " << type << " in line: " << line << std::endl;
            exit(1);
        }
    }

    Scene scene = Scene(*camera_ptr, shapes, lights);

    if (PRINT_MAIN) {
        print_parsed_scene(scene, objects);
    }
    return scene;
}

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " [scene_description_file.txt] [xres] [yres] [mode]" << std::endl;
        exit(1);
    }

    // Part 1 [Complete]
    // Parse the scene file
    // Recreate the shaded surface renderer from hw2 within an OpenGL context
    int xres = atoi(argv[2]);
    int yres = atoi(argv[3]);
    Scene scene = read_scene_file(argv[1]);
    scene.xres = xres;
    scene.yres = yres;
    set_scene(scene);

    /* 'glutInit' intializes the GLUT (Graphics Library Utility Toolkit) library.
     * This is necessary, since a lot of the functions we used above and below
     * are from the GLUT library.
     */
    glutInit(&argc, argv);
    /* The following line of code tells OpenGL that we need a double buffer,
     * a RGB pixel buffer, and a depth buffer.
     */
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("OpenGL Renderer");

    init();

    // Read shaders
    std::string vertProgFileName = "vertexProgram.glsl";
    std::string fragProgFileName = "fragmentProgram.glsl";
    ShadingType shading_type = atoi(argv[4]) == 0 ? GOURAUD : PHONG;

    // Do not run if using Gouraud shading
    if (shading_type != GOURAUD) {
        read_shaders(vertProgFileName, fragProgFileName);
    }

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    // Part 2
    // Add an Arcball implementation based on your own quaternion method
    glutMouseFunc(mouse_pressed);
    glutMotionFunc(mouse_moved);

    // Add keyboard to translate camera
    glutKeyboardFunc(key_pressed);

    glutMainLoop();

    
    return 0;
}