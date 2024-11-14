#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <cstdlib>
#include <vector>
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glu.h>
#include <GLUT/glut.h>

#include "utils.hpp"
#include "opengl_utils.hpp"
#include "arcball_utils.hpp"

Scene scene;
std::vector<OpenGL_Shape> opengl_shapes;
bool wireframe_mode = false;
GLenum shader_program = 0;
GLint nLightsUniformPos, camPosUniformPos;

// Mouse and keyboard Variables
int start_mouse_x, start_mouse_y;
float mouse_scale_x, mouse_scale_y;
const float step_size = 0.2;
float x_view_angle = 0;
bool is_pressed = false;

MyQuaternion last_rotation, current_rotation;

void set_scene(Scene& new_scene) {
    scene = new_scene;
}

void init() {

    // Reset the arcball
    last_rotation = MyQuaternion::Identity();
    current_rotation = MyQuaternion::Identity();

    // Gouraud shading
    glShadeModel(GL_SMOOTH);

    // Backface culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // Depth buffering
    glEnable(GL_DEPTH_TEST);

    // Normalize the surface normals
    glEnable(GL_NORMALIZE);

    // Here is where we store our vertices and surface normals
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    // Set up projection matrix (camera -> NDC)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glFrustum(scene.camera.l, scene.camera.r,
              scene.camera.b, scene.camera.t,
              scene.camera.n, scene.camera.f);

    // Switch to the modelview matrix
    glMatrixMode(GL_MODELVIEW);

    // Create objects from the scene
    create_objects_from_scene();

    // Initialize the lights from the scene
    init_lights();
}

void read_shaders(std::string vertex_shader_filename, std::string fragment_shader_filename) {
    std::string vertProgramSource, fragProgramSource;
   
    std::ifstream vertProgFile(vertex_shader_filename);
    if (! vertProgFile)
        std::cerr << "Error opening vertex shader program\n";
    std::ifstream fragProgFile(fragment_shader_filename);
    if (! fragProgFile)
        std::cerr << "Error opening fragment shader program\n";

    getline(vertProgFile, vertProgramSource, '\0');
    const char* vertShaderSource = vertProgramSource.c_str();

    getline(fragProgFile, fragProgramSource, '\0');
    const char* fragShaderSource = fragProgramSource.c_str();

    char buf[1024];
    GLsizei blah;

    // Initialize shaders
    GLenum vertShader, fragShader;

    shader_program = glCreateProgram();

    vertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertShader, 1, &vertShaderSource, NULL);
    glCompileShader(vertShader);
        
    GLint isCompiled = 0;
    glGetShaderiv(vertShader, GL_COMPILE_STATUS, &isCompiled);
    if(isCompiled == GL_FALSE)
    {
        GLint maxLength = 0;
        glGetShaderiv(vertShader, GL_INFO_LOG_LENGTH, &maxLength);
        
        // The maxLength includes the NULL character
        std::vector<GLchar> errorLog(maxLength);
        glGetShaderInfoLog(vertShader, maxLength, &maxLength, &errorLog[0]);
        
        // Provide the infolog in whatever manor you deem best.
        // Exit with failure.
        for (int i = 0; i < errorLog.size(); i++)
            std::cout << errorLog[i];
        glDeleteShader(vertShader); // Don't leak the shader.
        return;
    }

    fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragShader, 1, &fragShaderSource, NULL);
    glCompileShader(fragShader);

    isCompiled = 0;
    glGetShaderiv(fragShader, GL_COMPILE_STATUS, &isCompiled);
    if(isCompiled == GL_FALSE)
    {
        GLint maxLength = 0;
        glGetShaderiv(fragShader, GL_INFO_LOG_LENGTH, &maxLength);
        
        // The maxLength includes the NULL character
        std::vector<GLchar> errorLog(maxLength);
        glGetShaderInfoLog(fragShader, maxLength, &maxLength, &errorLog[0]);
        
        // Provide the infolog in whatever manor you deem best.
        // Exit with failure.
        for (int i = 0; i < errorLog.size(); i++)
            std::cout << errorLog[i];
        glDeleteShader(fragShader); // Don't leak the shader.
        return;
    }

    glAttachShader(shader_program, vertShader);
    glAttachShader(shader_program, fragShader);
    glLinkProgram(shader_program);
    std::cerr << "Enabling fragment program: " << gluErrorString(glGetError()) << std::endl;
    glGetProgramInfoLog(shader_program, 1024, &blah, buf);
    std::cerr << buf;

    std::cerr << "Enabling program object" << std::endl;
    glUseProgram(shader_program);

    nLightsUniformPos = glGetUniformLocation(shader_program, "nLights");
    camPosUniformPos = glGetUniformLocation(shader_program, "camPos");

    return;
}

void reshape(int width, int height) {
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;

    glViewport(0, 0, width, height);

    // Used for arcball stuff
    mouse_scale_x = (float) (scene.camera.r - scene.camera.l) / (float) width;
    mouse_scale_y = (float) (scene.camera.t - scene.camera.b) / (float) height;

    glutPostRedisplay();
}

Matrix4d get_current_matrix() {
    MyQuaternion q = current_rotation * last_rotation;
    q.normalize();
    
    double s = q.s;
    double x = q.v.x();
    double y = q.v.y();
    double z = q.v.z();

    Matrix4d R;
    R << 1 - 2 * (y * y + z * z), 2 * (x * y - s * z), 2 * (x * z + s * y), 0,
         2 * (x * y + s * z), 1 - 2 * (x * x + z * z), 2 * (y * z - s * x), 0,
         2 * (x * z - s * y), 2 * (y * z + s * x), 1 - 2 * (x * x + y * y), 0,
         0, 0, 0, 1;
    
    return R;
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glLoadIdentity();

    glRotatef(x_view_angle, 0, 1, 0);

    // World to Camera Coordinates
    glRotatef(-scene.camera.rot_angle * 180.0f / M_PI,
              scene.camera.rot_axis(0),
              scene.camera.rot_axis(1),
              scene.camera.rot_axis(2));
    glTranslatef(-scene.camera.position(0),
                 -scene.camera.position(1),
                 -scene.camera.position(2));

    // Arcball stuff
    Matrix4d R = get_current_matrix();
    glMultMatrixd(R.data());

    // Set lights
    set_lights();

    // Draw objects
    draw_objects();

    // Apply Shaders
    glUseProgram(shader_program);
    int nLights = scene.lights.size();
    glUniform1i(nLightsUniformPos, nLights);
    glUniform3f(camPosUniformPos, scene.camera.position(0), scene.camera.position(1), scene.camera.position(2));

    // Puts the off-screen buffer in front of the on-screen buffer
    glutSwapBuffers();
}

void init_lights() {
    glEnable(GL_LIGHTING);

    int num_lights = scene.lights.size();

    for (int i = 0; i < num_lights; i++) {
        int light_id = GL_LIGHT0 + i;

        glEnable(light_id);

        float color[3];
        color[0] = scene.lights[i].c.r;
        color[1] = scene.lights[i].c.g;
        color[2] = scene.lights[i].c.b;

        glLightfv(light_id, GL_AMBIENT, color);
        glLightfv(light_id, GL_DIFFUSE, color);
        glLightfv(light_id, GL_SPECULAR, color);

        glLightf(light_id, GL_QUADRATIC_ATTENUATION, scene.lights[i].k);
    }
}

void set_lights() {
    int num_lights = scene.lights.size();

    for (int i = 0; i < num_lights; i++) {
        int light_id = GL_LIGHT0 + i;

        // Position of the light
        float position[4];
        position[0] = scene.lights[i].position(0);
        position[1] = scene.lights[i].position(1);
        position[2] = scene.lights[i].position(2);
        position[3] = 1.0f;


        glLightfv(light_id, GL_POSITION, position);
    }
}

void draw_objects() {
    for (auto& shape : opengl_shapes) {
        // Adding a dupe of the current matrix so we can restore it later
        glPushMatrix();

        // Apply object transformations in reverse order
        for (int i = shape.transforms.size() - 1; i >= 0; i--) {
            TransformInfo t = shape.transforms[i];

            switch (t.type) {
                case TT_TRANSLATE:
                    glTranslatef(t.values[0], t.values[1], t.values[2]);
                    break;
                case TT_SCALE:
                    glScalef(t.values[0], t.values[1], t.values[2]);
                    break;
                case TT_ROTATE:
                    glRotatef(t.rotation_angle, t.values[0], t.values[1], t.values[2]);
                    break;
                default:
                    break;
            }
        }

        // Set material properties
        glMaterialfv(GL_FRONT, GL_AMBIENT, shape.ambient);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, shape.diffuse);
        glMaterialfv(GL_FRONT, GL_SPECULAR, shape.specular);
        glMaterialf(GL_FRONT, GL_SHININESS, shape.shininess);

        // Set vertex and normal buffer pointers
        glVertexPointer(3, GL_FLOAT, 0, &shape.vertex_buffer[0]);
        glNormalPointer(GL_FLOAT, 0, &shape.normal_buffer[0]);

        // Wireframe thingy from the tutorial (not sure if needed)
        int buffer_size = shape.vertex_buffer.size();

        if (!wireframe_mode) {
            glDrawArrays(GL_TRIANGLES, 0, buffer_size);
        } else {
            for(int j = 0; j < buffer_size; j += 3) {
                glDrawArrays(GL_LINE_LOOP, j, 3);
            }
        }
        
    }
}

void create_objects_from_scene() {
    for (auto& shape : scene.shapes) {
        OpenGL_Shape gl_shape;

        gl_shape.ambient[0] = shape.ambient.r;
        gl_shape.ambient[1] = shape.ambient.g;
        gl_shape.ambient[2] = shape.ambient.b;

        gl_shape.diffuse[0] = shape.diffuse.r;
        gl_shape.diffuse[1] = shape.diffuse.g;
        gl_shape.diffuse[2] = shape.diffuse.b;

        gl_shape.specular[0] = shape.specular.r;
        gl_shape.specular[1] = shape.specular.g;
        gl_shape.specular[2] = shape.specular.b;

        gl_shape.shininess = shape.shininess;

        for (auto& face : shape.faces) {
            Vector4f v1 = shape.points[face.v1].v;
            Vector4f v2 = shape.points[face.v2].v;
            Vector4f v3 = shape.points[face.v3].v;

            Vector3f n1 = shape.normals[face.n1];
            Vector3f n2 = shape.normals[face.n2];
            Vector3f n3 = shape.normals[face.n3];

            // Convert to triples
            Triple t1( v1(0), v1(1), v1(2) );
            Triple t2( v2(0), v2(1), v2(2) );
            Triple t3( v3(0), v3(1), v3(2) );

            Triple n1_t( n1(0), n1(1), n1(2) );
            Triple n2_t( n2(0), n2(1), n2(2) );
            Triple n3_t( n3(0), n3(1), n3(2) );

            gl_shape.vertex_buffer.push_back(t1);
            gl_shape.vertex_buffer.push_back(t2);
            gl_shape.vertex_buffer.push_back(t3);

            gl_shape.normal_buffer.push_back(n1_t);
            gl_shape.normal_buffer.push_back(n2_t);
            gl_shape.normal_buffer.push_back(n3_t);
        }

        gl_shape.transforms = shape.transforms;
        
        opengl_shapes.push_back(gl_shape);
    }
}

void mouse_pressed(int button, int state, int x, int y) {
    /* If the left-mouse button was clicked down, then...
     */
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        /* Store the mouse position in our global variables.
         */
        start_mouse_x = x;
        start_mouse_y = y;
        
        /* Since the mouse is being pressed down, we set our 'is_pressed"
         * boolean indicator to true.
         */
        is_pressed = true;
    }
    /* If the left-mouse button was released up, then...
     */
    else if(button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    {
        /* Mouse is no longer being pressed, so set our indicator to false.
         */
        is_pressed = false;
        last_rotation = current_rotation * last_rotation;
        current_rotation = MyQuaternion::Identity();
    }
}

void mouse_moved(int x, int y) {
    if(is_pressed) {
        Vector3d start = map_to_sphere(start_mouse_x, start_mouse_y, scene.xres, scene.yres);
        Vector3d curr = map_to_sphere(x, y, scene.xres, scene.yres);

        Vector3d u = start.cross(curr);
        if (u.norm() > 1e-6) {
            u.normalize();

            double angle = acos(std::min(1.0, start.dot(curr) / (start.norm() * curr.norm())));
            
            MyQuaternion q(cos(angle / 2), u * sin(angle / 2));
            // q.normalize();

            current_rotation = q;
        }
        
        glutPostRedisplay();
    }
}

void key_pressed(unsigned char key, int x, int y) {
    /* If 'q' is pressed, quit the program.
     */
    if(key == 'q')
    {
        exit(0);
    }
    /* If 't' is pressed, toggle our 'wireframe_mode' boolean to make OpenGL
     * render our cubes as surfaces of wireframes.
     */
    else if(key == 't')
    {
        wireframe_mode = !wireframe_mode;
        glutPostRedisplay();
    }
    else
    {
        float x_view_rad = x_view_angle * M_PI / 180.0;
        
        /* 'w' for step forward
         */
        if(key == 'w')
        {
            scene.camera.position[0] += step_size * sin(x_view_rad);
            scene.camera.position[2] -= step_size * cos(x_view_rad);
            glutPostRedisplay();
        }
        /* 'a' for step left
         */
        else if(key == 'a')
        {
            scene.camera.position[0] -= step_size * cos(x_view_rad);
            scene.camera.position[2] -= step_size * sin(x_view_rad);
            glutPostRedisplay();
        }
        /* 's' for step backward
         */
        else if(key == 's')
        {
            scene.camera.position[0] -= step_size * sin(x_view_rad);
            scene.camera.position[2] += step_size * cos(x_view_rad);
            glutPostRedisplay();
        }
        /* 'd' for step right
         */
        else if(key == 'd')
        {
            scene.camera.position[0] += step_size * cos(x_view_rad);
            scene.camera.position[2] += step_size * sin(x_view_rad);
            glutPostRedisplay();
        }
    }
}