#include "utils.hpp"
#include "structs.h"

#include <iostream>
#include <tuple>
#include <fstream>

bool PRINT_UTILS = false;
/*
############################
Color, Light, Shape, Point, & Face struct Methods
############################
*/
Color::Color(float r, float g, float b) {
    this->r = r;
    this->g = g;
    this->b = b;
}

Light::Light(float x, float y, float z, float r, float g, float b, float k) {
    this->position = Vector3f(x, y, z);
    this->c = Color(r, g, b);
    this->k = k;
}

Point::Point(Vector4f position) {
    this->v = position;
    this->c = Color(0, 0, 0);
}

Shape::Shape(
    std::string& name,
    std::string& filename,
    std::vector<HEF*> *hefs,
    std::vector<HEV*> *hevs,
    std::vector<TransformInfo>& transforms,
    Color& ambient,
    Color& diffuse,
    Color& specular,
    float shininess) {
    this->name = name;
    this->filename = filename;
    this->hefs = hefs;
    this->transforms = transforms;
    this->hevs = hevs;
    this->ambient = ambient;
    this->diffuse = diffuse;
    this->specular = specular;
    this->shininess = shininess;
}

/*
############################
Object struct Methods
############################
*/

/* 
 * `directory` ends with a slash
*/
Object::Object(std::string name, std::string filename, std::string directory) : name(name), filename(filename) {
        std::string file_path = directory + filename;
        std::ifstream file(file_path);

        if (file.fail()) {
            std::cerr << "Error opening file: " << file_path << std::endl;
            exit(1);
        }

        Mesh_Data *mesh_data = new Mesh_Data();
        mesh_data->faces = new std::vector<Face *>();
        mesh_data->vertices = new std::vector<Vertex *>();

        mesh_data->vertices->push_back(nullptr); // 1-indexed
        std::string line;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string type;
            iss >> type;
            if (type == "v") {
                float x, y, z;
                iss >> x >> y >> z;
                Vertex *v = new Vertex(x, y, z);
                mesh_data->vertices->push_back(v);
            } else if (type == "f") {
                std::vector<std::string> tokens;
                int v1, v2, v3;
                iss >> v1 >> v2 >> v3;
                Face *f = new Face(v1, v2, v3);
                mesh_data->faces->push_back(f);
            }
        }
        file.close();

        // calculate the normals using Halfedge data structure
        this->hevs = new std::vector<HEV *>();
        this->hefs = new std::vector<HEF *>();
        std::cout << "Before calling build_HE" << std::endl;
        build_HE(mesh_data, hevs, hefs);
        std::cout << "After calling build_HE" << std::endl;
}

/*
###############################
Camera struct Methods
###############################
*/

Camera::Camera(
    Vector3f& position, 
    Vector3f& rot_axis, 
    float rot_angle,
    float near, 
    float far, 
    float left, 
    float right, 
    float top, 
    float bottom) {
        this->position = position;
        this->rot_axis = rot_axis;
        this->rot_angle = rot_angle;
        this->n = near;
        this->f = far;
        this->l = left;
        this->r = right;
        this->t = top;
        this->b = bottom;
}

/*
###############################
Scene struct Methods
###############################
*/
Scene::Scene(Camera *camera, std::vector<Shape *> *shapes, std::vector<Light *> *lights) {
    this->camera = camera;
    this->shapes = shapes;
    this->lights = lights;
}

/* 
############################
Helper functions
############################
*/
Camera read_camera_section(std::ifstream& file) {
    Vector3f position;
    Vector3f rot_axis;
    float rot_angle;
    float near;
    float far;
    float left;
    float right;
    float top;
    float bottom;

    std::string line;
    while(std::getline(file, line) && !line.empty()) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "position") {
            float x, y, z;
            iss >> x >> y >> z;
            position = Vector3f(x, y, z);
        } else if (type == "orientation") {
            float x, y, z, theta;
            iss >> x >> y >> z >> theta;
            rot_axis = Vector3f(x, y, z);
            rot_angle = theta;
        } else if (type == "near") {
            iss >> near;
        } else if (type == "far") {
            iss >> far;
        } else if (type == "left") {
            iss >> left;
        } else if (type == "right") {
            iss >> right;
        } else if (type == "top") {
            iss >> top;
        } else if (type == "bottom") {
            iss >> bottom;
        }
    }

    return Camera(position, rot_axis, rot_angle, near, far, left, right, top, bottom);
};

/* 
 * `directory` ends with a slash
*/
void read_objects_section(std::ifstream& file, std::unordered_map<std::string, Object *> *objects, std::string directory) {
    std::string line;
    while(std::getline(file, line) && !line.empty()) {
        std::istringstream iss(line);
        std::string obj_name, obj_filename;
        iss >> obj_name >> obj_filename;
        objects->insert(std::make_pair(obj_name, new Object(obj_name, obj_filename, directory)));
    }
};

Shape create_shape(std::string name, std::string filename, std::ifstream& file, Object& obj) {
    std::string line;
    std::vector<TransformInfo> transforms;
    Color ambient = Color(0.0, 0.0, 0.0);
    Color diffuse = Color(0.0, 0.0, 0.0);
    Color specular = Color(0.0, 0.0, 0.0);
    float shininess = 0.0;
    while(std::getline(file, line) && !line.empty()) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "shininess") {
            iss >> shininess;
            continue;
        }

        TransformInfo transform;
        Vector3f v;
        float x, y, z;
        iss >> x >> y >> z;
        v << x, y, z;
        if (type == "ambient") {
            ambient = Color(v(0), v(1), v(2));
        } else if (type == "diffuse") {
            diffuse = Color(v(0), v(1), v(2));
        } else if (type == "specular") {
            specular = Color(v(0), v(1), v(2));
        } else {
            if (type == "t") {
                transform.type = TT_TRANSLATE;
            } else if (type == "r") {
                transform.type = TT_ROTATE;
                float angle;
                iss >> angle;
                angle = angle * 180 / M_PI; // convert from radians to degrees
                transform.rotation_angle = angle;
            } else if (type == "s") {
                transform.type = TT_SCALE;
            }
            transform.values[0] = v(0);
            transform.values[1] = v(1);
            transform.values[2] = v(2);
            transforms.push_back(transform);
        }
    }

    return Shape(name, filename, obj.hefs, obj.hevs, transforms, ambient, diffuse, specular, shininess);
}

Vector4f transform_to_ndc(Matrix4f& space_matrix, Matrix4f& pers_matrix, Vector4f v) {
    Matrix4f T = pers_matrix * space_matrix; // Converts from world coordinates to normalized device coordinates
       
    Vector4f new_v = T * v;
    new_v /= new_v(3); // divide by w_ndc
    
    return new_v;
}
