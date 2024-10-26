/*

Task: write a program in C++ that takes as input a file with the below format
and outputs a PPM image of the specified scene (with desired x and y
resolutions)
File format example:

camera:
position 0 0 5  
orientation 0 1 0 0  
near 1  
far 10  
left -1  
right 1  
top 1  
bottom -1  
 
objects:  
cube cube.obj  
cube2 cube2.obj  
 
cube  
s 1 1 1  
r 0 0 1 0  
t -3 -3 -1  
 
cube2  
s 2 -2 2  
r 0 0 1 0  
t -3 3 -1  
 
cube2  
s 1 1 1  
r 0 0 1 0  
t -1.5 -1.5 -0.5  
s -2 -2 2  
r 0 0 1 0  
t 0 0 0  
 
cube  
s 1 1 1  
r 0 0 1 0  
t -3 -3 -1  
s -1 1 1  
r 0 0 1 0  
t 0 0 0

*/

#include "utils.hpp"
#include <iostream>
#include <fstream>

Camera read_camera_section(std::ifstream& file) {
    Vector3d position;
    Vector3d rot_axis;
    double rot_angle;
    double near;
    double far;
    double left;
    double right;
    double top;
    double bottom;

    std::string line;
    while(std::getline(file, line) && !line.empty()) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "position") {
            double x, y, z;
            iss >> x >> y >> z;
            position = Vector3d(x, y, z);
        } else if (type == "orientation") {
            double x, y, z, theta;
            iss >> x >> y >> z >> theta;
            rot_axis = Vector3d(x, y, z);
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

void read_objects_section(std::ifstream& file, std::unordered_map<std::string, Object>& obj_transforms) {
    std::string line;
    while(std::getline(file, line) && !line.empty()) {
        std::istringstream iss(line);
        std::string obj_name, obj_filename;
        iss >> obj_name >> obj_filename;
        obj_transforms.insert(std::make_pair(obj_name, Object(obj_name, obj_filename)));
    }
};

void read_transform(std::ifstream& file, Object& obj) {
    std::string line;
    Matrix4d product = Matrix4d::Identity();
    while(std::getline(file, line) && !line.empty()) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        Matrix4d M;
        Vector3d v;
        double x, y, z;
        iss >> x >> y >> z;
        v << x, y, z;
        if (type == "t") {
            calc_translation_matrix(v, M);
        } else if (type == "r") {
            double angle;
            iss >> angle;
            calc_rotation_matrix(v, angle, M);
        } else if (type == "s") {
            calc_scaling_matrix(v, M);
        }

        product = M * product;
    }

    obj.transform_matrix.push_back(product);
};

void print_part_1(Scene& scene) {
    // make sure camera is correct by printing all field in Camera object
    Camera *camera = scene.camera;
    std::cout << "Camera position: " << camera->position.transpose() << std::endl;
    std::cout << "Camera rotation axis: " << camera->rot_axis.transpose() << std::endl;
    std::cout << "Camera rotation angle: " << camera->rot_angle << std::endl;
    std::cout << "Camera near: " << camera->n << std::endl;
    std::cout << "Camera far: " << camera->f << std::endl;
    std::cout << "Camera left: " << camera->l << std::endl;
    std::cout << "Camera right: " << camera->r << std::endl;
    std::cout << "Camera top: " << camera->t << std::endl;
    std::cout << "Camera bottom: " << camera->b << std::endl;

    // make sure objects are correct
    std::unordered_map<std::string, Object> obj_transforms = scene.objects_map;
    for (auto& obj : obj_transforms) {
        std::cout << "Object name: " << obj.first << std::endl;
        std::cout << "Object filename: " << obj.second.filename << std::endl;
        std::cout << "Object transforms: " << std::endl;
        for (auto& transform : obj.second.transform_matrix) {
            std::cout << transform << std::endl;
        }
        std::cout << "Object vertices: " << std::endl;
        for (auto& v : obj.second.vertices) {
            std::cout << v(0) << " " << v(1) << " " << v(2)  << std::endl;
        }
        std::cout << "Object faces: " << std::endl;
        for (auto& face : obj.second.faces) {
            std::cout << face.v1 << " " << face.v2 << " " << face.v3 << std::endl;
        }
    }
}

Scene read_scene_file(std::string filename) {
    Camera *camera = nullptr;
    std::unordered_map<std::string, Object> obj_transforms;

    std::ifstream file(filename);
    std::string line;
    while(std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "camera:") {
            camera = new Camera(read_camera_section(file));
        } else if (type == "objects:") {
            read_objects_section(file, obj_transforms);
        } else if (obj_transforms.find(type) != obj_transforms.end()) {
            read_transform(file, obj_transforms.at(type));
        } else {
            std::cerr << "Unknown type: " << type << " in line: " << line << std::endl;
            exit(1);
        }
    }

    return Scene(camera, obj_transforms);
};

void print_transformed_vectors(Scene& scene) {
    std::cout << "++++++++++++++++" << std::endl;
    std::cout << "Transformed vectors:" << std::endl;
    for (auto& obj : scene.objects_map) {
        for (auto& vset: obj.second.transformed_vertices) {
            for (auto& v : vset) {
                std::cout << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << std::endl;
            }
        }
    }
    std::cout << "++++++++++++++++" << std::endl;
}

void print_cartesian_ndc(Scene& scene) {
    std::cout << "Cartesian vectors:" << std::endl;
    for (auto& obj : scene.objects_map) {
        for (auto& vset_ptr: obj.second.cartesian_ndc) {
            for (auto& v_ptr : vset_ptr) {
                if (v_ptr == nullptr) {
                    continue;
                }
                Vector4d& v = *v_ptr;
                std::cout << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << std::endl;
            }
        }
    }
    std::cout << "++++++++++++++++" << std::endl;
}

void print_screen_vectors(Scene& scene) {
    std::cout << "Screen vectors:" << std::endl;
    for (auto& obj : scene.objects_map) {
        for (auto& vset_ptr: obj.second.screen_coords) {
            for (auto& v_ptr : vset_ptr) {
                if (v_ptr == nullptr) {
                    continue;
                }
                Vector2d& v = *v_ptr;
                std::cout << v(0) << " " << v(1) << std::endl;
            }
        }
    }
    std::cout << "++++++++++++++++" << std::endl;
}

int main(int argc, char** argv) {
    bool PRINT = false;
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " [scene_description_file.txt] [xres] [yres]" << std::endl;
        return 1;
    }

    // Part 1 & Part 2
    Scene scene = read_scene_file(argv[1]);
    int xres = atoi(argv[2]);
    int yres = atoi(argv[3]);

    if (PRINT) {
        std::cout << "world_to_camera: " << std::endl;
        std::cout << scene.space_matrix << std::endl;
        std::cout << "perspective: " << std::endl;
        std::cout << scene.pers_matrix << std::endl;
    }

    if (PRINT) {
        print_part_1(scene);
    }

    // Part 3
    for (auto& obj : scene.objects_map) {
        obj.second.apply_transforms_and_cartesian_ndc(scene.space_matrix, scene.pers_matrix);
    }

    if (PRINT) {
        print_transformed_vectors(scene);
        print_cartesian_ndc(scene);
    }

    // Part 4
    for (auto& obj : scene.objects_map) {
        obj.second.apply_screen_mapping(xres, yres);
    }

    if (PRINT) {
        print_screen_vectors(scene);
    }

    int col_res = xres;
    int row_res = yres;
    // Part 5
    std::vector<std::vector<bool>> img(row_res, std::vector<bool>(col_res, false));
    for (auto& obj : scene.objects_map) {
        Object *obj_ptr = &obj.second;
        for (auto& obj_copy_ptr : obj_ptr->screen_coords) {
            for (auto& face : obj_ptr->faces) {
                
                bool v1_on_screen = obj_copy_ptr[face.v1 - 1] != nullptr;
                bool v2_on_screen = obj_copy_ptr[face.v2 - 1] != nullptr;
                bool v3_on_screen = obj_copy_ptr[face.v3 - 1] != nullptr;
                
                if (v1_on_screen && v2_on_screen) {
                    const Vector2d& v1 = *obj_copy_ptr[face.v1 - 1];
                    const Vector2d& v2 = *obj_copy_ptr[face.v2 - 1];
                    draw_line(v1(0), v1(1), v2(0), v2(1), img);
                }

                if (v2_on_screen && v3_on_screen) {
                    const Vector2d& v2 = *obj_copy_ptr[face.v2 - 1];
                    const Vector2d& v3 = *obj_copy_ptr[face.v3 - 1];
                    draw_line(v2(0), v2(1), v3(0), v3(1), img);
                }

                if (v3_on_screen && v1_on_screen) {
                    const Vector2d& v1 = *obj_copy_ptr[face.v1 - 1];
                    const Vector2d& v3 = *obj_copy_ptr[face.v3 - 1];
                    draw_line(v3(0), v3(1), v1(0), v1(1), img);
                }
            }
        }
    }


    // Part 6 output to standard output as a PPM image
    std::cout << "P3" << std::endl;
    std::cout << xres << " " << yres << std::endl;
    std::cout << "255" << std::endl;
    for (int i = 0; i < row_res; i++) {
        for (int j = 0; j < col_res; j++) {
            if (img[i][j]) {
                std::cout << "255 255 255" << std::endl;
            } else {
                std::cout << "0 0 0" << std::endl;
            }
        }
    }

    return 0;
};
