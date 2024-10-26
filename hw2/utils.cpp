#include "utils.hpp"

#include <iostream>
#include <fstream>

bool PRINT_UTILS = false;
/*
############################
Color, Light, Shape, Point, & Face struct Methods
############################
*/
Color::Color(double r, double g, double b) {
    this->r = r;
    this->g = g;
    this->b = b;
}

Light::Light(double x, double y, double z, double r, double g, double b, double k) {
    this->position = Vector3d(x, y, z);
    this->c = Color(r, g, b);
    this->k = k;
}

Point::Point(Vector4d position) {
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
    std::vector<Vector4d>& world_vertices,
    std::vector<Vector3d>& normals,
    Color& ambient,
    Color& diffuse,
    Color& specular,
    double shininess) {
    this->name = name;
    this->filename = filename;
    this->faces = faces;
    for (auto& v : world_vertices) {
        this->points.push_back(Point(v));
    }
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
                double x, y, z;
                iss >> x >> y >> z;
                vertices.push_back(Vector4d(x, y, z, 1.0));
            } else if (type == "vn") {
                double x, y, z;
                iss >> x >> y >> z;
                surface_normals.push_back(Vector3d(x, y, z));
            } else if (type == "f") {
                std::vector<std::string> tokens;
                std::string token;
                while (iss >> token) {
                    tokens.push_back(token);
                }
                double v1, v2, v3;
                double n1, n2, n3;
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
Scene::Scene(Camera& camera, std::vector<Shape>& shapes, std::vector<Light>& lights) {
    this->camera = camera;
    this->shapes = shapes;
    this->lights = lights;
    // Part 2
    calc_space_transform_matrix();
    calc_pers_matrix();
}

void Scene::calc_space_transform_matrix() {
    Matrix4d trans_matrix;
    calc_translation_matrix(camera.position, trans_matrix);
    Matrix4d rot_matrix;
    calc_rotation_matrix(camera.rot_axis, camera.rot_angle, rot_matrix);

    space_matrix = (trans_matrix * rot_matrix).inverse();
}

void Scene::calc_pers_matrix() {
    double n = camera.n;
    double f = camera.f;
    double r = camera.r;
    double l = camera.l;
    double t = camera.t;
    double b = camera.b;
    Matrix4d orig_matrix;
    orig_matrix << 2*n/(r-l), 0,         (r+l)/(r-l),  0,
                   0,         2*n/(t-b), (t+b)/(t-b),  0,
                   0,         0,         -(f+n)/(f-n), -2*f*n/(f-n),
                   0,         0,         -1,           0;

    pers_matrix = orig_matrix;
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
    // assert that u is a unit vector
    u.normalize();
    assert(u.norm() - 1 < 1e-6);

    // calculate the rotation matrix
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
    Matrix4d product = Matrix4d::Identity();
    Matrix4d surface_norms_product = Matrix4d::Identity();
    Color ambient = Color(0.0, 0.0, 0.0);
    Color diffuse = Color(0.0, 0.0, 0.0);
    Color specular = Color(0.0, 0.0, 0.0);
    double shininess = 0.0;
    while(std::getline(file, line) && !line.empty()) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "shininess") {
            iss >> shininess;
            continue;
        }

        Matrix4d M;
        Vector3d v;
        double x, y, z;
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
                calc_translation_matrix(v, M);
            } else if (type == "r") {
                double angle;
                iss >> angle;
                calc_rotation_matrix(v, angle, M);
            } else if (type == "s") {
                calc_scaling_matrix(v, M);
            }
            product = M * product;

            if (type != "t") {
                surface_norms_product = M * surface_norms_product;
            }
        }
    }

    surface_norms_product = surface_norms_product.inverse().transpose();
    std::vector<Vector4d> world_vertices;
    std::vector<Vector3d> normals;
    if (PRINT_UTILS) {
        std::cout << "This is the transformation matrix: \n" << product << std::endl;
        std::cout << "This is the surface normal transformation matrix: \n" << surface_norms_product << std::endl;
        std::cout << "Transformed Vertices: " << std::endl;
    }
    for (auto& v : obj.vertices) {
        world_vertices.push_back(product * v);
        if (PRINT_UTILS) {
            std::cout << (product * v).transpose() << std::endl;
        }
    }
    if (PRINT_UTILS) {
        std::cout << "Transformed Surface normals: " << std::endl;
    }
    for (auto& n : obj.surface_normals) {
        Vector4d fourd_n;
        fourd_n << n(0), n(1), n(2), 1.0;
        Vector4d norm_trans = surface_norms_product * fourd_n;
        Vector3d normalized = Vector3d(norm_trans(0), norm_trans(1), norm_trans(2)).normalized();
        normals.push_back(normalized);
        if (PRINT_UTILS) {
            std::cout << normalized.transpose() << std::endl;
        }
    }

    return Shape(name, filename, obj.faces, world_vertices, normals, ambient, diffuse, specular, shininess);
}

Vector4d transform_to_ndc(Matrix4d& space_matrix, Matrix4d& pers_matrix, Vector4d v) {
    Matrix4d T = pers_matrix * space_matrix; // Converts from world coordinates to normalized device coordinates
       
    Vector4d new_v = T * v;
    new_v /= new_v(3); // divide by w_ndc
    
    return new_v;
}

// Part 4
void calc_lighting_model(
    Color ambient_c,
    Color diffuse_c,
    Color specular_c,
    double shininess_phong,
    Point& p,
    Vector3d& n,
    std::vector<Light>& lights,
    Vector3d& cam_position) {
        // n.normalize();
        if (PRINT_UTILS) {
            std::cout << n.transpose() << ", norm: " << n.norm() << ", diff: " << std::abs(n.norm() - 1.0) <<  ", bool: " << (std::abs(n.norm() - 1.0) < 1e-6) << std::endl;
        }
        assert(std::abs(n.norm() - 1.0) < 1e-6);
        Vector3d ambient = Vector3d(ambient_c.r, ambient_c.g, ambient_c.b);
        Vector3d diffuse = Vector3d(diffuse_c.r, diffuse_c.g, diffuse_c.b);
        Vector3d specular = Vector3d(specular_c.r, specular_c.g, specular_c.b);

        Vector3d diffuse_sum = Vector3d(0.0, 0.0, 0.0);
        Vector3d specular_sum = Vector3d(0.0, 0.0, 0.0);

        Vector3d v = Vector3d(p.v(0), p.v(1), p.v(2));
        Vector3d cam_dir = (cam_position - v).normalized();

        for (auto& l : lights) {
            Vector3d light_pos = l.position;
            Vector3d light_c = Vector3d(l.c.r, l.c.g, l.c.b);
            Vector3d light_dir = (light_pos - v).normalized();

            double d = (v - light_pos).norm();
            double attenuation = 1 / (1 + l.k * std::pow(d, 2));
            light_c *= attenuation;

            Vector3d light_diffuse = light_c * std::max(0.0, n.dot(light_dir));
            diffuse_sum += light_diffuse;

            double spec_val = std::max(0.0, n.dot((cam_dir + light_dir).normalized()));
            Vector3d light_specular = light_c * std::pow(spec_val, shininess_phong);
            specular_sum += light_specular;
        }

        Vector3d result = ambient + diffuse_sum.cwiseProduct(diffuse) + specular_sum.cwiseProduct(specular);
        Vector3d new_c = Vector3d::Ones().cwiseMin(result);

        p.c = Color(new_c(0), new_c(1), new_c(2));
}

double barycentric_function(Vector2d& i, Vector2d& j, double x, double y) {
    return (i.y() - j.y()) * x + (j.x() - i.x()) * y + i.x() * j.y() - j.x() * i.y();
}

std::tuple<double, double, double> calc_barycentric_coords(
    Vector2d& a,
    Vector2d& b,
    Vector2d& c,
    int x,
    int y) {
    double alpha = barycentric_function(b, c, x, y) / barycentric_function(b, c, a.x(), a.y());
    double beta = barycentric_function(a, c, x, y) / barycentric_function(a, c, b.x(), b.y());
    double gamma = barycentric_function(a, b, x, y) / barycentric_function(a, b, c.x(), c.y());

    return std::make_tuple(alpha, beta, gamma);
}

Vector2d map_screen(Vector4d& v,int xres, int yres) {
    /*
    This will make the points be like how they should appear on the screen,
    this means that (0,0) corresponds to the top left corner. However, when
    inputting to an image array, need to flip x and y coords since there the
    first idx represents vertical and second idx represents horizontal.
    */
    double new_x = (xres - 1) * (1 + v(0)) / 2;
    double new_y = (yres - 1) * (1 - v(1)) / 2;

    return Vector2d(new_x, new_y);
}

bool valid_coords(double alpha, double beta, double gamma) {
    return alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1;
}

bool in_cube(Vector3d& v_ndc) {
    return (v_ndc(0) > -1 && v_ndc(0) < 1 
            && v_ndc(1) > -1 && v_ndc(1) < 1
            && v_ndc(2) > -1 && v_ndc(2) < 1);
}

void rasterize_triangle_gouraud(
    Point& p1_ndc,
    Point& p2_ndc,
    Point& p3_ndc,
    int xres,
    int yres,
    std::vector<std::vector<Color>>& img,
    std::vector<std::vector<double>>& depth_buffer) {
    /*
    Implements depth buffering, note that `depth_buffer` should be
    initialized with infinity values.

    Implements backface culling which will only draw triangles that are facing the camera.

    Input coordinates are ndc coordinates and we have to convert them to screen
    coordinates ourselves.

    We are still in the assumption here that x and y are like in cartesian, not (column, row)
    like on screen. Because of this, when we draw, we draw img[y][x]
    */
    Vector3d cross = (p3_ndc.v.head<3>() - p2_ndc.v.head<3>()).cross(p1_ndc.v.head<3>() - p2_ndc.v.head<3>());

    // if triangle is facing away, backface culling stops it from being rendered.
    if (cross(2) < 0) {
        return;
    }

    Vector2d p1_screen = map_screen(p1_ndc.v, xres, yres);
    Vector2d p2_screen = map_screen(p2_ndc.v, xres, yres);
    Vector2d p3_screen = map_screen(p3_ndc.v, xres, yres);
    if (PRINT_UTILS) {
        std::cout << "These are the screen vectors:" << std::endl;
        std::cout << p1_screen.transpose() << std::endl;
        std::cout << p2_screen.transpose() << std::endl;
        std::cout << p3_screen.transpose() << std::endl;
    }
    
    Color c1 = p1_ndc.c;
    Color c2 = p2_ndc.c;
    Color c3 = p3_ndc.c;

    int min_x = std::min(p1_screen(0), std::min(p2_screen(0), p3_screen(0)));
    int max_x = std::max(p1_screen(0), std::max(p2_screen(0), p3_screen(0)));
    int min_y = std::min(p1_screen(1), std::min(p2_screen(1), p3_screen(1)));
    int max_y = std::max(p1_screen(1), std::max(p2_screen(1), p3_screen(1)));

    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            std::tuple<double, double, double> bary = 
                calc_barycentric_coords(p1_screen, p2_screen, p3_screen, x, y);
            double alpha = std::get<0>(bary);
            double beta = std::get<1>(bary);
            double gamma = std::get<2>(bary);

            Vector3d v_ndc = (p1_ndc.v * alpha + p2_ndc.v * beta + p3_ndc.v * gamma).head<3>();
            double point_z = v_ndc.z();

            if (valid_coords(alpha, beta, gamma) && in_cube(v_ndc) && point_z <= depth_buffer[y][x]) {
                depth_buffer[y][x] = point_z;
                double r = alpha * c1.r + beta * c2.r + gamma * c3.r;
                double g = alpha * c1.g + beta * c2.g + gamma * c3.g;
                double b = alpha * c1.b + beta * c2.b + gamma * c3.b;
                img[y][x] = Color(r, g, b);
            }
        }
    }
}

void rasterize_triangle_phong(
    Point& p1,
    Point& p2,
    Point& p3,
    Vector3d& n1,
    Vector3d& n2,
    Vector3d& n3,
    Shape& shape,
    Matrix4d& pers_matrix,
    Matrix4d& space_matrix,
    std::vector<Light>& lights,
    Vector3d& cam_pos,
    int xres,
    int yres,
    std::vector<std::vector<Color>>& img,
    std::vector<std::vector<double>>& depth_buffer) {
    /*
    Takes in world coordinates
    Implements backface culling
    */

    Point p1_ndc = transform_to_ndc(space_matrix, pers_matrix, p1.v);
    Point p2_ndc = transform_to_ndc(space_matrix, pers_matrix, p2.v);
    Point p3_ndc = transform_to_ndc(space_matrix, pers_matrix, p3.v);

    Vector3d cross = (p3_ndc.v.head<3>() - p2_ndc.v.head<3>()).cross(p1_ndc.v.head<3>() - p2_ndc.v.head<3>());

    // if triangle is facing away, backface culling stops it from being rendered.
    if (cross(2) < 0) {
        return;
    }

    Vector2d p1_screen = map_screen(p1_ndc.v, xres, yres);
    Vector2d p2_screen = map_screen(p2_ndc.v, xres, yres);
    Vector2d p3_screen = map_screen(p3_ndc.v, xres, yres);
    Color c1 = p1_ndc.c;
    Color c2 = p2_ndc.c;
    Color c3 = p3_ndc.c;

    int min_x = std::min(p1_screen(0), std::min(p2_screen(0), p3_screen(0)));
    int max_x = std::max(p1_screen(0), std::max(p2_screen(0), p3_screen(0)));
    int min_y = std::min(p1_screen(1), std::min(p2_screen(1), p3_screen(1)));
    int max_y = std::max(p1_screen(1), std::max(p2_screen(1), p3_screen(1)));

    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            std::tuple<double, double, double> bary = 
                calc_barycentric_coords(p1_screen, p2_screen, p3_screen, x, y);
            double alpha = std::get<0>(bary);
            double beta = std::get<1>(bary);
            double gamma = std::get<2>(bary);

            Vector3d v_ndc = (p1_ndc.v * alpha + p2_ndc.v * beta + p3_ndc.v * gamma).head<3>();
            double point_z = v_ndc.z();

            if (valid_coords(alpha, beta, gamma) && in_cube(v_ndc) && point_z <= depth_buffer[y][x]) {
                depth_buffer[y][x] = point_z;
                Vector3d n = (alpha * n1 + beta * n2 + gamma * n3).normalized();
                Vector4d v = p1.v * alpha + p2.v * beta + p3.v * gamma;
                Point p(v);
                calc_lighting_model(
                    shape.ambient,
                    shape.diffuse,
                    shape.specular,
                    shape.shininess,
                    p,
                    n,
                    lights,
                    cam_pos);

                img[y][x] = p.c;
            }
        }
    }
}

void fill_grid(Scene scene, int xres, int yres, std::vector<std::vector<Color>>& img, ShadingType shading_type) {
    std::vector<std::vector<double>> depth_buffer(yres, std::vector<double>(xres, std::numeric_limits<double>::max()));
    for (auto& sh : scene.shapes) {
        for (auto& f : sh.faces) {
            Point p1 = sh.points[f.v1];
            Point p2 = sh.points[f.v2];
            Point p3 = sh.points[f.v3];
            Vector3d n1 = sh.normals[f.n1];
            Vector3d n2 = sh.normals[f.n2];
            Vector3d n3 = sh.normals[f.n3];
            
            if (shading_type == GOURAUD) {
                Point p1_ndc = transform_to_ndc(scene.space_matrix, scene.pers_matrix, p1.v);
                Point p2_ndc = transform_to_ndc(scene.space_matrix, scene.pers_matrix, p2.v);
                Point p3_ndc = transform_to_ndc(scene.space_matrix, scene.pers_matrix, p3.v);

                calc_lighting_model(
                    sh.ambient,
                    sh.diffuse,
                    sh.specular,
                    sh.shininess,
                    p1,
                    n1,
                    scene.lights,
                    scene.camera.position);
                calc_lighting_model(
                    sh.ambient,
                    sh.diffuse,
                    sh.specular,
                    sh.shininess,
                    p2,
                    n2,
                    scene.lights,
                    scene.camera.position);
                calc_lighting_model(
                    sh.ambient,
                    sh.diffuse,
                    sh.specular,
                    sh.shininess,
                    p3,
                    n3,
                    scene.lights,
                    scene.camera.position);

                p1_ndc.c = p1.c;
                p2_ndc.c = p2.c;
                p3_ndc.c = p3.c;

                rasterize_triangle_gouraud(p1_ndc, p2_ndc, p3_ndc, xres, yres, img, depth_buffer);
            }
            else if (shading_type == PHONG) {
                rasterize_triangle_phong(
                    p1,
                    p2,
                    p3,
                    n1,
                    n2,
                    n3,
                    sh,
                    scene.pers_matrix,
                    scene.space_matrix,
                    scene.lights,
                    scene.camera.position,
                    xres,
                    yres,
                    img,
                    depth_buffer);
            }
        }
    }
}

void print_ppm(std::vector<std::vector<Color>>& img, int xres, int yres, int color_size) {
    std::cout << "P3" << std::endl;
    std::cout << xres << " " << yres << std::endl;
    std::cout << color_size << std::endl;

    int row_res = img.size();
    int col_res = img[0].size();

    for (int i = 0; i < row_res; i++) {
        for (int j = 0; j < col_res; j++) {
            int r = int(img[i][j].r * color_size);
            int g = int(img[i][j].g * color_size);
            int b = int(img[i][j].b * color_size);
            std::cout << r << " " << g << " " << b << " " << std::endl;
        }
    }
}
