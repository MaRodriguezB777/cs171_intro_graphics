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
    // Part 2
    calc_space_transform_matrix();
    calc_pers_matrix();
}

void Scene::calc_space_transform_matrix() {
    Matrix4f trans_matrix;
    calc_translation_matrix(camera.position, trans_matrix);
    Matrix4f rot_matrix;
    calc_rotation_matrix(camera.rot_axis, camera.rot_angle, rot_matrix);

    space_matrix = (trans_matrix * rot_matrix).inverse();
}

void Scene::calc_pers_matrix() {
    float n = camera.n;
    float f = camera.f;
    float r = camera.r;
    float l = camera.l;
    float t = camera.t;
    float b = camera.b;
    Matrix4f orig_matrix;
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
void calc_translation_matrix(Vector3f& v, Matrix4f& T) {
    T << 1, 0, 0, v(0),
         0, 1, 0, v(1),
         0, 0, 1, v(2),
         0, 0, 0, 1;
}

void calc_rotation_matrix(Vector3f& u, float angle, Matrix4f& R) {
    // assert that u is a unit vector
    u.normalize();
    assert(u.norm() - 1 < 1e-6);

    // calculate the rotation matrix
    float ux2 = u(0) * u(0);
    float uy2 = u(1) * u(1);
    float uz2 = u(2) * u(2);
    float uxy = u(0) * u(1);
    float uxz = u(0) * u(2);
    float uyz = u(1) * u(2);
    float ux = u(0);
    float uy = u(1);
    float uz = u(2);

    R << ux2 + (1 - ux2) * cos(angle), uxy * (1 - cos(angle)) - uz * sin(angle), uxz * (1 - cos(angle)) + uy * sin(angle), 0,
         uxy * (1 - cos(angle)) + uz * sin(angle), uy2 + (1 - uy2) * cos(angle), uyz * (1 - cos(angle)) - ux * sin(angle), 0,
         uxz * (1 - cos(angle)) - uy * sin(angle), uyz * (1 - cos(angle)) + ux * sin(angle), uz2 + (1 - uz2) * cos(angle), 0,
         0, 0, 0, 1;
}

void calc_scaling_matrix(Vector3f& v, Matrix4f& S) {
    S << v(0), 0, 0, 0,
         0, v(1), 0, 0,
         0, 0, v(2), 0,
         0, 0, 0, 1;
}

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

// Part 4
void calc_lighting_model(
    Color ambient_c,
    Color diffuse_c,
    Color specular_c,
    float shininess_phong,
    Point& p,
    Vector3f& n,
    std::vector<Light>& lights,
    Vector3f& cam_position) {
        // n.normalize();
        if (PRINT_UTILS) {
            std::cout << n.transpose() << ", norm: " << n.norm() << ", diff: " << std::abs(n.norm() - 1.0) <<  ", bool: " << (std::abs(n.norm() - 1.0) < 1e-6) << std::endl;
        }
        assert(std::abs(n.norm() - 1.0) < 1e-6);
        Vector3f ambient = Vector3f(ambient_c.r, ambient_c.g, ambient_c.b);
        Vector3f diffuse = Vector3f(diffuse_c.r, diffuse_c.g, diffuse_c.b);
        Vector3f specular = Vector3f(specular_c.r, specular_c.g, specular_c.b);

        Vector3f diffuse_sum = Vector3f(0.0, 0.0, 0.0);
        Vector3f specular_sum = Vector3f(0.0, 0.0, 0.0);

        Vector3f v = Vector3f(p.v(0), p.v(1), p.v(2));
        Vector3f cam_dir = (cam_position - v).normalized();

        for (auto& l : lights) {
            Vector3f light_pos = l.position;
            Vector3f light_c = Vector3f(l.c.r, l.c.g, l.c.b);
            Vector3f light_dir = (light_pos - v).normalized();

            float d = (v - light_pos).norm();
            double attenuation = 1 / (1 + l.k * std::pow(d, 2));
            light_c *= attenuation;

            Vector3f light_diffuse = light_c * std::max(0.0, (double)(n.dot(light_dir)));
            diffuse_sum += light_diffuse;

            double spec_val = std::max(0.0, (double)(n.dot((cam_dir + light_dir).normalized())));
            Vector3f light_specular = light_c * std::pow(spec_val, shininess_phong);
            specular_sum += light_specular;
        }

        Vector3f result = ambient + diffuse_sum.cwiseProduct(diffuse) + specular_sum.cwiseProduct(specular);
        Vector3f new_c = Vector3f::Ones().cwiseMin(result);

        p.c = Color(new_c(0), new_c(1), new_c(2));
}

float barycentric_function(Vector2f& i, Vector2f& j, float x, float y) {
    return (i.y() - j.y()) * x + (j.x() - i.x()) * y + i.x() * j.y() - j.x() * i.y();
}

std::tuple<float, float, float> calc_barycentric_coords(
    Vector2f& a,
    Vector2f& b,
    Vector2f& c,
    int x,
    int y) {
    float alpha = barycentric_function(b, c, x, y) / barycentric_function(b, c, a.x(), a.y());
    float beta = barycentric_function(a, c, x, y) / barycentric_function(a, c, b.x(), b.y());
    float gamma = barycentric_function(a, b, x, y) / barycentric_function(a, b, c.x(), c.y());

    return std::make_tuple(alpha, beta, gamma);
}

Vector2f map_screen(Vector4f& v,int xres, int yres) {
    /*
    This will make the points be like how they should appear on the screen,
    this means that (0,0) corresponds to the top left corner. However, when
    inputting to an image array, need to flip x and y coords since there the
    first idx represents vertical and second idx represents horizontal.
    */
    float new_x = (xres - 1) * (1 + v(0)) / 2;
    float new_y = (yres - 1) * (1 - v(1)) / 2;

    return Vector2f(new_x, new_y);
}

bool valid_coords(float alpha, float beta, float gamma) {
    return alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1;
}

bool in_cube(Vector3f& v_ndc) {
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
    std::vector<std::vector<Color> >& img,
    std::vector<std::vector<float> >& depth_buffer) {
    /*
    Implements depth buffering, note that `depth_buffer` should be
    initialized with infinity values.

    Implements backface culling which will only draw triangles that are facing the camera.

    Input coordinates are ndc coordinates and we have to convert them to screen
    coordinates ourselves.

    We are still in the assumption here that x and y are like in cartesian, not (column, row)
    like on screen. Because of this, when we draw, we draw img[y][x]
    */
    Vector3f cross = (p3_ndc.v.head<3>() - p2_ndc.v.head<3>()).cross(p1_ndc.v.head<3>() - p2_ndc.v.head<3>());

    // if triangle is facing away, backface culling stops it from being rendered.
    if (cross(2) < 0) {
        return;
    }

    Vector2f p1_screen = map_screen(p1_ndc.v, xres, yres);
    Vector2f p2_screen = map_screen(p2_ndc.v, xres, yres);
    Vector2f p3_screen = map_screen(p3_ndc.v, xres, yres);
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
            std::tuple<float, float, float> bary = 
                calc_barycentric_coords(p1_screen, p2_screen, p3_screen, x, y);
            float alpha = std::get<0>(bary);
            float beta = std::get<1>(bary);
            float gamma = std::get<2>(bary);

            Vector3f v_ndc = (p1_ndc.v * alpha + p2_ndc.v * beta + p3_ndc.v * gamma).head<3>();
            float point_z = v_ndc.z();

            if (valid_coords(alpha, beta, gamma) && in_cube(v_ndc) && point_z <= depth_buffer[y][x]) {
                depth_buffer[y][x] = point_z;
                float r = alpha * c1.r + beta * c2.r + gamma * c3.r;
                float g = alpha * c1.g + beta * c2.g + gamma * c3.g;
                float b = alpha * c1.b + beta * c2.b + gamma * c3.b;
                img[y][x] = Color(r, g, b);
            }
        }
    }
}

void rasterize_triangle_phong(
    Point& p1,
    Point& p2,
    Point& p3,
    Vector3f& n1,
    Vector3f& n2,
    Vector3f& n3,
    Shape& shape,
    Matrix4f& pers_matrix,
    Matrix4f& space_matrix,
    std::vector<Light>& lights,
    Vector3f& cam_pos,
    int xres,
    int yres,
    std::vector<std::vector<Color> >& img,
    std::vector<std::vector<float> >& depth_buffer) {
    /*
    Takes in world coordinates
    Implements backface culling
    */

    Point p1_ndc = transform_to_ndc(space_matrix, pers_matrix, p1.v);
    Point p2_ndc = transform_to_ndc(space_matrix, pers_matrix, p2.v);
    Point p3_ndc = transform_to_ndc(space_matrix, pers_matrix, p3.v);

    Vector3f cross = (p3_ndc.v.head<3>() - p2_ndc.v.head<3>()).cross(p1_ndc.v.head<3>() - p2_ndc.v.head<3>());

    // if triangle is facing away, backface culling stops it from being rendered.
    if (cross(2) < 0) {
        return;
    }

    Vector2f p1_screen = map_screen(p1_ndc.v, xres, yres);
    Vector2f p2_screen = map_screen(p2_ndc.v, xres, yres);
    Vector2f p3_screen = map_screen(p3_ndc.v, xres, yres);
    Color c1 = p1_ndc.c;
    Color c2 = p2_ndc.c;
    Color c3 = p3_ndc.c;

    int min_x = std::min(p1_screen(0), std::min(p2_screen(0), p3_screen(0)));
    int max_x = std::max(p1_screen(0), std::max(p2_screen(0), p3_screen(0)));
    int min_y = std::min(p1_screen(1), std::min(p2_screen(1), p3_screen(1)));
    int max_y = std::max(p1_screen(1), std::max(p2_screen(1), p3_screen(1)));

    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            std::tuple<float, float, float> bary = 
                calc_barycentric_coords(p1_screen, p2_screen, p3_screen, x, y);
            float alpha = std::get<0>(bary);
            float beta = std::get<1>(bary);
            float gamma = std::get<2>(bary);

            Vector3f v_ndc = (p1_ndc.v * alpha + p2_ndc.v * beta + p3_ndc.v * gamma).head<3>();
            float point_z = v_ndc.z();

            if (valid_coords(alpha, beta, gamma) && in_cube(v_ndc) && point_z <= depth_buffer[y][x]) {
                depth_buffer[y][x] = point_z;
                Vector3f n = (alpha * n1 + beta * n2 + gamma * n3).normalized();
                Vector4f v = p1.v * alpha + p2.v * beta + p3.v * gamma;
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

void fill_grid(Scene scene, int xres, int yres, std::vector<std::vector<Color> >& img, ShadingType shading_type) {
    std::vector<std::vector<float> > depth_buffer(yres, std::vector<float>(xres, std::numeric_limits<float>::max()));
    for (auto& sh : scene.shapes) {
        for (auto& f : sh.faces) {
            Point p1 = sh.points[f.v1];
            Point p2 = sh.points[f.v2];
            Point p3 = sh.points[f.v3];
            Vector3f n1 = sh.normals[f.n1];
            Vector3f n2 = sh.normals[f.n2];
            Vector3f n3 = sh.normals[f.n3];
            
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

void print_ppm(std::vector<std::vector<Color> >& img, int xres, int yres, int color_size) {
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
