#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <string>

struct Vertex {
    float x, y, z;
};

struct Face {
    int v1, v2, v3;
};

struct Object {
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
};

Object read_obj_file(const std::string& filename) {
    Object obj;
    std::ifstream file(filename);

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "v") {
            Vertex v;
            iss >> v.x >> v.y >> v.z;
            obj.vertices.push_back(v);
        } else if (type == "f") {
            Face f;
            iss >> f.v1 >> f.v2 >> f.v3;
            obj.faces.push_back(f);
        }
    }

    file.close();
    return obj;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " obj_file1.obj obj_file2.obj ... obj_fileN.obj" << std::endl;
        return 1;
    }

    for (int i = 1; i < argc; i++) {
        std::string filename  = argv[i];
        Object obj = read_obj_file(filename);

        std::cout << filename << ":\n" << std::endl;
        for (auto& v : obj.vertices) {
            std::cout << "v " << v.x << ' ' << v.y << ' ' << v.z << std::endl;
        }
        for (auto& f : obj.faces) {
            std::cout << "f " << f.v1 << ' ' << f.v2 << ' ' << f.v3 << std::endl;
        }
        std::cout << std::endl;
    }
}