#include "utils.hpp"

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

Face::Face(int v1, int v2, int v3, int n1, int n2, int n3) {
    this->v1 = v1;
    this->v2 = v2;
    this->v3 = v3;
    this->n1 = n1;
    this->n2 = n2;
    this->n3 = n3;
}

Shape::Shape(   
    std::string& name,
    std::string& filename,
    std::vector<Face>& faces,
    std::vector<Vector4f>& world_vertices,
    std::vector<Vector3f>& normals,
    std::vector<TransformInfo>& transforms,
    Color& ambient,
    Color& diffuse,
    Color& specular,
    float shininess) {
    this->name = name;
    this->filename = filename;
    this->faces = faces;
    for (auto& v : world_vertices) {
        this->points.push_back(Point(v));
    }
    this->transforms = transforms;
    this->normals = normals;
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
Object::Object(std::string name, std::string filename) : name(name), filename(filename) {
        std::string file_path = "data/" + filename;
        std::ifstream file(file_path);
        
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string type;
            iss >> type;
            if (type == "v") {
                float x, y, z;
                iss >> x >> y >> z;
                vertices.push_back(Vector4f(x, y, z, 1.0));
            } else if (type == "vn") {
                float x, y, z;
                iss >> x >> y >> z;
                surface_normals.push_back(Vector3f(x, y, z));
            } else if (type == "f") {
                std::vector<std::string> tokens;
                std::string token;
                while (iss >> token) {
                    tokens.push_back(token);
                }
                float v1, v2, v3;
                float n1, n2, n3;
                for (int i = 0; i < 3; i++) {
                    std::string t = tokens[i];
                    int slash_idx = t.find("//");
                    switch(i) {
                        case 0:
                            v1 = std::stod(t.substr(0, slash_idx));
                            n1 = std::stod(t.substr(slash_idx+2));
                            break;
                        case 1:
                            v2 = std::stod(t.substr(0, slash_idx));
                            n2 = std::stod(t.substr(slash_idx+2));
                            break;
                        case 2:
                            v3 = std::stod(t.substr(0, slash_idx));
                            n3 = std::stod(t.substr(slash_idx+2));
                            break;
                        default:
                            break;
                    }
                }
                // Make 0-indexed
                faces.push_back(Face(v1 - 1, v2 - 1, v3 - 1, n1 - 1, n2 - 1, n3 - 1));
            }
        }
        file.close();
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
Scene::Scene(Camera& camera, std::vector<Shape>& shapes, std::vector<Light>& lights) {
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

void read_objects_section(std::ifstream& file, std::unordered_map<std::string, Object>& objects) {
    std::string line;
    while(std::getline(file, line) && !line.empty()) {
        std::istringstream iss(line);
        std::string obj_name, obj_filename;
        iss >> obj_name >> obj_filename;
        objects.insert(std::make_pair(obj_name, Object(obj_name, obj_filename)));
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

    return Shape(name, filename, obj.faces, obj.vertices, obj.surface_normals, transforms, ambient, diffuse, specular, shininess);
}

Vector4f transform_to_ndc(Matrix4f& space_matrix, Matrix4f& pers_matrix, Vector4f v) {
    Matrix4f T = pers_matrix * space_matrix; // Converts from world coordinates to normalized device coordinates
       
    Vector4f new_v = T * v;
    new_v /= new_v(3); // divide by w_ndc
    
    return new_v;
}
