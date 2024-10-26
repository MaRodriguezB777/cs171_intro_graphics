#include "utils.hpp"

#include <iostream>
#include <fstream>

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

void Object::apply_geometric_transforms() {
    for (auto& T : transform_matrices) {
        std::vector<Vector4d> new_vertices;
        for (auto& v : vertices) {
            new_vertices.push_back(T * v);
        }
        transformed_vertices.push_back(new_vertices);
    }
}

std::vector<std::vector<Vector4d*>> Object::get_cartesian_ndc(Matrix4d& S, Matrix4d& P) {
    Matrix4d T = P * S; // Converts from world coordinates to normalized device coordinates
    
    std::vector<std::vector<Vector4d*>> result;
    for (auto& vset : transformed_vertices) {
        std::vector<Vector4d*> new_vertices;
        for (auto& v : vset) {
            Vector4d* new_v = new Vector4d(T * v);
            *new_v /= (*new_v)(3);
            // std::cout << "This is the homogenous ndc: " << std::endl;
            // std::cout << (*new_v)(0) << " " << (*new_v)(1) << " " << (*new_v)(2) << " " << (*new_v)(3) << std::endl;
            
            // Removed because it messes up the faces mapping
            if ((*new_v)(0) < -1 || (*new_v)(0) > 1 || (*new_v)(1) < -1 || (*new_v)(1) > 1) {
                new_vertices.push_back(nullptr);
                delete new_v;
            } else {
                new_vertices.push_back(new_v);
            }
        }
        result.push_back(new_vertices);
    }

    return result;
}

void Object::apply_transforms_and_cartesian_ndc(Matrix4d& space_matrix, Matrix4d& pers_matrix) {
    // Part 3
    apply_geometric_transforms();
    std::vector<std::vector<Vector4d*>> trans_objs = get_cartesian_ndc(space_matrix, pers_matrix);

    for (auto& vset_ptr : trans_objs) {
        cartesian_ndc.push_back(vset_ptr);
    }
}

void Object::apply_screen_mapping(int xres, int yres) {
    // Part 4
    for (auto& vset_ptr : cartesian_ndc) {
        std::vector<Vector2d*> new_vset;
        for (auto& v_ptr : vset_ptr) {
            if (v_ptr == nullptr) {
                new_vset.push_back(nullptr);
                continue;
            }
            Vector4d& v = *v_ptr;
            double new_x = (xres - 1) * (1 - v(0)) / 2;
            double new_y = (yres - 1) * (1 + v(1)) / 2;
            new_vset.push_back(new Vector2d(new_x, new_y));
        }
        screen_coords.push_back(new_vset);
    }
}

/* 
############################
Helper functions
############################
*/
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

void draw_line(int x0, int y0, int x1, int y1, std::vector<std::vector<bool>>& img) {
    /* 
    Generalized Bresenham Line Algorithm explanation:

    All lines in the plane can be categorized by two parameters:
    1. Whether the slope is positive or negative
    2. Whether the line is steep or not (|m| > 1 or |m| < 1)
    3. Whether we are drawing left to right or right to left

    For the drawing direction, we can choose one direction and WLOG
    we choose drawing left to right. This generalizes any lines going
    into the 3rd through 6th quadrant.

    Now we handle the remaining 4 quadrants:

    Quadrant 1. Provided in notes
    Quadrant 8. This case is similar to Quadrant 1, except that error
    accumulates downward, therefore we have to correct it by adding dx. Also,
    we need to switch the sign of the y increment and reverse the inequality
    since dy is now negative.
    Quadrant 2. Here, because we have a steep, increasing line, we can flip the
    x and y and treat this the same as quadrant 1.
    Quadrant 7. Here, similarly to Quadrant 2, we flip the x and y and treat
    this the same as quadrant 8.
    */
    // draw from left to right
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dx = x1 - x0;
    int dy = y1 - y0;

    bool steep = abs(dy) > abs(dx);
    int err = 0;

    if (!steep) { // iterate over x
        int y = y0;
        if (dy >= 0) { // (Quadrant 1) drawing upward
            for (int x = x0; x <= x1; x++) {
                color_pixel(x, y, img);
                if (2 * (err + dy) < dx) {
                    err += dy;
                } else {
                    err += dy - dx;
                    y++;
                }
            }
        } else { // (Quadrant 8) drawing downward
            for (int x = x0; x <= x1; x++) {
                color_pixel(x, y, img);
                if (2 * (err + dy) > dx) {
                    err += dy;
                } else {
                    err += dy + dx;
                    y--;
                }
            }
        }
    } else { // iterate over y
        int x = x0;
        if (dy >= 0) { // (Quadrant 2) drawing upward
            for (int y = y0; y <= y1; y++) {
                color_pixel(x, y, img);
                if (2 * (err + dx) < dy) {
                    err += dx;
                } else {
                    err += dx - dy;
                    x++;
                }
            }
        } else { // (Quadrant 7) drawing downward
            for (int y = y0; y >= y1; y--) {
                color_pixel(x, y, img);
                if (2 * (err + dx) > dy) {
                    err -= dx;
                } else {
                    err += -dx - dy;
                    x++;
                }
            }
        }
    }
}

void color_pixel(int x, int y, std::vector<std::vector<bool>>& img) {
    if (x < 0 || x >= img[0].size() || y < 0 || y >= img.size()) return;
    img[y][x] = true;
}

/*
###############################
Camera struct Methods
###############################
*/

Camera::Camera(
    Vector3d& position, 
    Vector3d& rot_axis, 
    double rot_angle,
    double near, 
    double far, 
    double left, 
    double right, 
    double top, 
    double bottom) {
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
Scene::Scene(Camera *camera, std::unordered_map<std::string, Object>& obj_transforms) {
    this->camera = camera;
    this->objects_map = obj_transforms;
    // Part 2
    calc_space_transform_matrix();
    calc_pers_matrix();
}

void Scene::calc_space_transform_matrix() {
    Matrix4d trans_matrix;
    calc_translation_matrix(camera->position, trans_matrix);
    Matrix4d rot_matrix;
    calc_rotation_matrix(camera->rot_axis, camera->rot_angle, rot_matrix);

    space_matrix = (trans_matrix * rot_matrix).inverse();
}


void Scene::calc_pers_matrix() {
    double n = -camera->n;
    double f = -camera->f;
    double r = camera->r;
    double l = camera->l;
    double t = camera->t;
    double b = camera->b;
    Matrix4d orig_matrix;
    orig_matrix << 2*n/(r-l), 0, (r+l)/(r-l), 0,
                    0, 2*n/(t-b), (t+b)/(t-b), 0,
                    0, 0, -(f+n)/(f-n), -2*f*n/(f-n),
                    0, 0, -1, 0;

    pers_matrix = orig_matrix;
}
