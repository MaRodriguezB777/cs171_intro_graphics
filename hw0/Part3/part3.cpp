/* 
We introduce a file format that puts together multiple 3d models (what is inside
.obj files) and associates each model with a set of transformations.)

File Example:
object1 object1_filename.obj  
object2 object2_filename.obj  
object3 object3_filename.obj  
 
object1  
t tx ty tz  
r rx ry rz angle_in_radians  
 
object1  
t tx ty tz  
s sx sy sz  
 
object2  
r rx ry rz angle_in_radians  
s sx sy sz  
 
object3  
t tx ty tz  
r rx ry rz angle_in_radians  
s sx sy sz

Steps to use this file:
1. Load in data from .obj files and associate with the corresponding labels
2. For each set of data, apply the transformations to the object
3. Output the transformed copies of each of the objects
...
- Outputs the transformed copies of each of the objects
*/

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

using namespace Eigen;

struct Face {
    int v1, v2, v3;
};

struct Object {
    std::string name;
    std::string filename;
    std::vector<Vector4d> vertices;
    std::vector<Face> faces;
    std::vector<Matrix4d> transforms;
    std::vector<std::vector<Vector4d> > transformed_vertices;

    Object(std::string name, std::string filename) : name(name), filename(filename) {
        std::ifstream file(filename);

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string type;
            iss >> type;
            if (type == "v") {
                double x, y, z;
                iss >> x >> y >> z;
                vertices.push_back(Vector4d(x, y, z, 1.0));
            } else if (type == "f") {
                Face f;
                iss >> f.v1 >> f.v2 >> f.v3;
                faces.push_back(f);
            }
        }
        file.close();
    }

    void apply_transforms() {
        for (auto& T : transforms) {
            std::vector<Vector4d> new_vertices;
            for (auto& v : vertices) {
                new_vertices.push_back(T * v);
            }
            transformed_vertices.push_back(new_vertices);
        }
    }
};

void calc_translation_matrix(Vector3d& v, Matrix4d& T) {
    T << 1, 0, 0, v(0),
         0, 1, 0, v(1),
         0, 0, 1, v(2),
         0, 0, 0, 1;
}

void calc_rotation_matrix(Vector3d& u, double angle, Matrix4d& R) {
    double ux2 = u(0) * u(0);
    double uy2 = u(1) * u(1);
    double uz2 = u(2) * u(2);
    double uxy = u(0) * u(1);
    double uxz = u(0) * u(2);
    double uyz = u(1) * u(2);
    double ux = u(0);
    double uy = u(1);
    double uz = u(2);

    R << ux2 + (1 - ux2) * cos(angle), uxy * (1 - cos(angle)) - uz * sin(angle), uxz * (1 - cos(angle)) + uy * sin(angle), 0,
         uxy * (1 - cos(angle)) + uz * sin(angle), uy2 + (1 - uy2) * cos(angle), uyz * (1 - cos(angle)) - ux * sin(angle), 0,
         uxz * (1 - cos(angle)) - uy * sin(angle), uyz * (1 - cos(angle)) + ux * sin(angle), uz2 + (1 - uz2) * cos(angle), 0,
         0, 0, 0, 1;
}

void calc_scaling_matrix(Vector3d& v, Matrix4d& S) {
    S << v(0), 0, 0, 0,
         0, v(1), 0, 0,
         0, 0, v(2), 0,
         0, 0, 0, 1;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " file.txt" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::ifstream file(filename);
    std::string line;
    std::unordered_map<std::string, Object> objects_by_name;
    std::vector<std::string> object_names;

    // define the objects
    while (std::getline(file, line)) {
        if (line.empty()) {
            break;
        }
        std::istringstream iss(line);

        std::string obj_name;
        std::string file_name;
        iss >> obj_name >> file_name;
        Object obj(obj_name, file_name);
        objects_by_name.insert(std::make_pair(obj_name, obj));
        object_names.push_back(obj_name);
        // objects.push_back(obj);
    }

    // create the copies of the objects
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string obj_name;
        iss >> obj_name;
        
        Matrix4d product = Matrix4d::Identity();
        while (std::getline(file, line) && !line.empty()) {
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

        // if obj_name is the map, then add the product to the transforms of the
        // object
        if (objects_by_name.find(obj_name) != objects_by_name.end()) {
            objects_by_name.at(obj_name).transforms.push_back(product);
        }

        // for (auto& obj : objects) {
        //     if (obj.name == obj_name) {
        //         obj.transforms.push_back(product);
        //     }
        // }
    }

    file.close();

    // output the transformed objects
    for (auto& obj : objects_by_name) {
        obj.second.apply_transforms();
    }
    // for (auto& obj : objects) {
    //     obj.apply_transforms();
    // }

    for (auto& obj_name : object_names) {
        Object obj = objects_by_name.at(obj_name);
        for (int i = 0; i < obj.transforms.size(); i++) {
            std::cout << obj_name << "_copy" << (i + 1) << std::endl;
            std::vector<Vector4d> t_vertices = obj.transformed_vertices[i];
            for (auto& v : t_vertices) {
                std::cout << v(0) << ' ' << v(1) << ' ' << v(2) << std::endl;
            }
            std::cout << std::endl;
        }
    }
    // for (auto& obj : objects) {
    //     for (int i = 0; i < obj.transforms.size(); i++) {
    //         std::cout << obj.name << "_copy" << (i + 1) << std::endl;
    //         std::vector<Vector4d> t_vertices = obj.transformed_vertices[i];
    //         for (auto& v : t_vertices) {
    //             std::cout << v(0) << ' ' << v(1) << ' ' << v(2) << std::endl;
    //         }
    //         std::cout << std::endl;
    //     }
    // }

    return 0;
}