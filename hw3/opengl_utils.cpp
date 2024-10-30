#include <GL/glew.h>
#include <GLUT/glut.h>
#include <vector>

#include "utils.hpp"
#include "opengl_utils.hpp"

Scene scene;
std::vector<OpenGL_Shape> opengl_shapes;
bool wireframe_mode = false;

void set_scene(Scene& new_scene) {
    scene = new_scene;
}

void init() {

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

void reshape(int width, int height) {
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;

    glViewport(0, 0, width, height);

    glutPostRedisplay();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glLoadIdentity();

    glRotatef(-scene.camera.rot_angle * 180.0f / M_PI,
              scene.camera.rot_axis(0),
              scene.camera.rot_axis(1),
              scene.camera.rot_axis(2));
    glTranslatef(-scene.camera.position(0),
                 -scene.camera.position(1),
                 -scene.camera.position(2));

    // Set lights
    set_lights();

    // Draw objects
    draw_objects();

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
