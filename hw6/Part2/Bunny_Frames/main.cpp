#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include "Eigen/Dense"

namespace fs = std::__fs::filesystem;
using namespace Eigen;

int n_frames_global = 0;
std::string obj_name = "bunny";

// For file comparison
double epsilonf = 1.02e-6;
int epsiloni = 1;

struct Vertex {
    float x, y, z;

    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}
    Vertex() {}
};

struct Frame {
    int n_frame;
    std::vector<Vertex *> *vertices;

    Frame(int frame) {
        this->n_frame = frame;
        this->vertices = new std::vector<Vertex *>();
    }
};

struct Frames {
    std::vector<Frame *> *interpolated_frames;
    std::vector<Frame *> *keyframes;
    std::vector<std::array<int, 3>> *faces;

    Frames() {
        this->interpolated_frames = new std::vector<Frame *>();
        this->keyframes = new std::vector<Frame *>();
        this->faces = new std::vector<std::array<int, 3>>();
    }
};

Frames *frames = new Frames();

void read_object_file(std::string file_path) {
    std::ifstream file(file_path);

    if (file.fail()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        exit(1);
    }

    // Assumes filepath like 'path/nameXX.obj'
    int idx = file_path.find_last_of('/');

    int n_frame = atoi(file_path.substr(idx + obj_name.length() + 1, 2).c_str());
    if (n_frame > n_frames_global) {
        n_frames_global = n_frame;
    }

    Frame *frame = new Frame(n_frame);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "v") {
            float x, y, z;
            iss >> x >> y >> z;
            Vertex *v = new Vertex(x, y, z);
            frame->vertices->push_back(v);
        }
        // Faces do not change across keyframes
        else if (type == "f" and n_frame == 0) {
            int v1, v2, v3;
            iss >> v1 >> v2 >> v3;
            frames->faces->push_back({v1, v2, v3});
        }
    }
    file.close();

    frames->keyframes->push_back(frame);
}

void read_keyframes(std::string directory) {
    std::vector<fs::path> files_in_directory;
    std::copy(fs::directory_iterator(directory), fs::directory_iterator(), std::back_inserter(files_in_directory));
    std::sort(files_in_directory.begin(), files_in_directory.end());


    for (auto& file : files_in_directory) {
        std::cout << "reading the file: " << file.string() << std::endl;
        read_object_file(file.string());
    }
}

float calcCatmullRomSpline(float p1, float p2, float p3, float p4, double u) {
    Matrix4d B = Eigen::Matrix4d::Zero();
    B <<  0.0,  1.0,  0.0,  0.0,
         -0.5,  0.0,  0.5,  0.0,
          1.0, -2.5,  2.0, -0.5,
         -0.5,  1.5, -1.5,  0.5;

    Vector4d u_vec(1.0, u, std::pow(u, 2.0), std::pow(u, 3.0));
    Vector4d p_vec(p1, p2, p3, p4);

    return u_vec.dot(B * p_vec);
}

void calcCatmullRomSpline(Vertex *p1, Vertex *p2, Vertex *p3, Vertex *p4, double u, float *result) {
    result[0] = calcCatmullRomSpline(p1->x, p2->x, p3->x, p4->x, u);
    result[1] = calcCatmullRomSpline(p1->y, p2->y, p3->y, p4->y, u);
    result[2] = calcCatmullRomSpline(p1->z, p2->z, p3->z, p4->z, u);
}

void computeTransformation(int vertex_idx, int current_frame, Vertex *new_vertex) {
    int n_keyframes = frames->keyframes->size();

    int kfi0_idx = -1;
    int kfi1_idx = -1;
    for (int i = 0; i < n_keyframes; i++) {
        Frame *kf = frames->keyframes->at(i);
        if (kf->n_frame <= current_frame) {
            kfi0_idx = i;
        }
        if (kf->n_frame > current_frame) {
            kfi1_idx = i;
            break;
        }
    }

    int kfi2_idx = (kfi1_idx + 1) % n_keyframes;
    int kfiMinus1_idx = (kfi0_idx + n_keyframes - 1) % n_keyframes;

    Frame *kfiMinus1 = frames->keyframes->at(kfiMinus1_idx);
    Frame *kfi0 = frames->keyframes->at(kfi0_idx);
    Frame *kfi1 = frames->keyframes->at(kfi1_idx);
    Frame *kfi2 = frames->keyframes->at(kfi2_idx);

    if (current_frame < 5) {
        kfiMinus1 = frames->keyframes->at(0);
        kfi0 = frames->keyframes->at(0);
    }
    if (current_frame > 15) {
        kfi1 = frames->keyframes->at(n_keyframes - 1);
        kfi2 = frames->keyframes->at(n_keyframes - 1);
    }

    int frame_range = (kfi1->n_frame - kfi0->n_frame + n_frames_global) % n_frames_global;
    int offset = (current_frame - kfi0->n_frame + n_frames_global) % n_frames_global;
    double u = 1.0 * offset / frame_range;
    
    Vertex *viMinus1 = kfiMinus1->vertices->at(vertex_idx);
    Vertex *vi0 = kfi0->vertices->at(vertex_idx);
    Vertex *vi1 = kfi1->vertices->at(vertex_idx);
    Vertex *vi2 = kfi2->vertices->at(vertex_idx);

    float result[3];
    calcCatmullRomSpline(viMinus1,
                         vi0,
                         vi1,
                         vi2,
                         u,
                         result);

    new_vertex->x = result[0];
    new_vertex->y = result[1];
    new_vertex->z = result[2];
}

void interpolate_keyframes(std::string interpolated_dir) {
    int n_vertices = frames->keyframes->at(0)->vertices->size();
    for (int frame_idx = 0; frame_idx < n_frames_global; frame_idx++) {
        
        bool isKeyFrame = false;
        for (auto& kf : *frames->keyframes) {
            if (kf->n_frame == frame_idx) {
                isKeyFrame = true;
                break;
            }
        }
        if (isKeyFrame) {
            continue;
        }

        Frame *interpolated_frame = new Frame(frame_idx);
        for (int v_idx = 0; v_idx < n_vertices; v_idx++) {
            Vertex *new_vertex = new Vertex();
            computeTransformation(v_idx, frame_idx, new_vertex);

            interpolated_frame->vertices->push_back(new_vertex);
        }
        frames->interpolated_frames->push_back(interpolated_frame);
    }

    std::cout << "Writing to file" << std::endl;
    std::cout << "Currently have " << frames->interpolated_frames->size() << " interpolated frames" << std::endl;

    for (auto& interpolated_frame : *frames->interpolated_frames) {
        // may need to convert to string
        int frame = interpolated_frame->n_frame;
        
        std::string frame_str = std::to_string(frame);
        if (frame < 10) {
            frame_str = "0" + frame_str;
        }

        std::string filename = interpolated_dir + frame_str + ".obj";
        std::ofstream file(filename);

        for (auto& vertex : *interpolated_frame->vertices) {
            file << "v " << vertex->x << " " << vertex->y << " " << vertex->z << std::endl;
        }

        for (auto& face : *frames->faces) {
            file << "f " << face[0] << " " << face[1] << " " << face[2] << std::endl;
        }

        file.close();
    }
}

void compare_files(std::string exp_filepath, std::string actual_filepath) {
    std::ifstream expected_file(exp_filepath);
    std::ifstream actual_file(actual_filepath);
    
    if (expected_file.fail()) {
        std::cerr << "Error opening file: " << exp_filepath << std::endl;
        exit(1);
    }
    if (actual_file.fail()) {
        std::cerr << "Error opening file: " << actual_filepath << std::endl;
        exit(1);
    }

    int line_count = 0;
    std::string exp_line, actual_line;
    while (std::getline(expected_file, exp_line) && std::getline(actual_file, actual_line)) {
        line_count++;
        std::istringstream exp_iss(exp_line);
        std::istringstream actual_iss(actual_line);

        std::string exp_type;
        exp_iss >> exp_type;
        std::string actual_type;
        actual_iss >> actual_type;

        if (exp_type != actual_type) {
            std::cout << "Type mismatch on line " << line_count;
            std::cout << "Expected: " << exp_type << ", Actual: " << actual_type << std::endl;
            exit(1);
        }

        if (exp_type == "v") {
            float exp_x, exp_y, exp_z;
            exp_iss >> exp_x >> exp_y >> exp_z;

            float actual_x, actual_y, actual_z;
            actual_iss >> actual_x >> actual_y >> actual_z;

            bool diff_x = std::abs(exp_x - actual_x) <= epsilonf;
            bool diff_y = std::abs(exp_y - actual_y) <= epsilonf;
            bool diff_z = std::abs(exp_z - actual_z) <= epsilonf;

            if (!(diff_x and diff_y and diff_z)) {
                std::cout << "Vertex mismatch on line " << line_count << std::endl;
                std::cout << "Epsilon: " << epsilonf << " Expected: " << exp_x << " " << exp_y << " " << exp_z << ", Actual: " << actual_x << " " << actual_y << " " << actual_z << std::endl;
                std::cout << "Diffs: " << std::abs(exp_x - actual_x) << " " << std::abs(exp_y - actual_y) << " " << std::abs(exp_z - actual_z) << std::endl;
                exit(1);
            }
        }
        // Faces do not change across keyframes
        else if (exp_type == "f") {
            int exp_v1, exp_v2, exp_v3;
            exp_iss >> exp_v1 >> exp_v2 >> exp_v3;

            int actual_v1, actual_v2, actual_v3;
            actual_iss >> actual_v1 >> actual_v2 >> actual_v3;

            bool diff1 = std::abs(exp_v1 - actual_v1) < epsiloni;
            bool diff2 = std::abs(exp_v2 - actual_v2) < epsiloni;
            bool diff3 = std::abs(exp_v3 - actual_v3) < epsiloni;

            if (!(diff1 and diff2 and diff3)) {
                std::cout << "Vertex mismatch on line " << line_count << std::endl;
                std::cout << "Expected: " << exp_v1 << " " << exp_v2 << " " << exp_v3 << ", Actual: " << actual_v1 << " " << actual_v2 << " " << actual_v3 << std::endl;
                exit(1);
            }
        }
    }

    if (std::getline(expected_file, exp_line)) {
        std::cout << "Expected file has more lines than actual file." << std::endl;
        exit(1);
    }
    if (std::getline(actual_file, actual_line)) {
        std::cout << "Actual file has more lines than expected file." << std::endl;
        exit(1);
    }

    std::cout << "Files are the same!" << std::endl;
}

void diff_expected_and_actual(std::string expected_dir, std::string actual_dir) {
    std::vector<fs::path> files_exp_dir;
    std::copy(fs::directory_iterator(expected_dir), fs::directory_iterator(), std::back_inserter(files_exp_dir));
    std::sort(files_exp_dir.begin(), files_exp_dir.end());

    std::vector<fs::path> files_actual_dir;
    std::copy(fs::directory_iterator(actual_dir), fs::directory_iterator(), std::back_inserter(files_actual_dir));
    std::sort(files_actual_dir.begin(), files_actual_dir.end());

    if (files_actual_dir.size() != files_exp_dir.size()) {
        std::cout << "Number of files is not even the same!";
        return;
    }

    for (int i = 0; i < files_exp_dir.size(); i++) {
        std::string exp_file = files_exp_dir.at(i);
        std::string actual_file = files_actual_dir.at(i);

        std::string exp_filename = exp_file.substr(exp_file.find_last_of('/'));
        std::string actual_filename = actual_file.substr(actual_file.find_last_of('/'));
        if (exp_filename != actual_filename) {
            std::cout << exp_file << " and " << actual_file << " aren't the same name!" << std::endl;
            continue;
        }

        std::cout << "Checking file: " << exp_file << std::endl;
        compare_files(exp_file, actual_file);
    }
}

int main(int argc, char* argv[]) {
    std::string directory = "./keyframes";

    read_keyframes(directory);

    std::string interpolated_dir = "./my_interpolated_frames/";
    interpolate_keyframes(interpolated_dir + "bunny");

    std::string expected_dir = "./interpolated_frames/";
    diff_expected_and_actual(expected_dir, interpolated_dir);
}