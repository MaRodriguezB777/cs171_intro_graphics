/* This is a code snippet for drawing the "I-bar". The 'quadratic' object can be made
 * into a global variable in your program if you want. Line 13, where 'quadratic' gets
 * initialized should be done in your 'init' function or somewhere close to the start
 * of your program.
 *
 * The I-bar is an object that Prof. Al Barr once used in one of his papers to demonstrate
 * an interpolation technique. You might call it "Al Barr's I-Bar" ;)
 */

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <cstdlib>
#include <vector>
#include <GLUT/glut.h>

#include "Eigen/Dense"
#include "utils.h"
#include "arcball_utils.hpp"

using namespace Eigen;

bool PRINT = true;

// The script to follow
Script *script = new Script();
int current_frame = 0;

/* Needed to draw the cylinders using glu */
GLUquadricObj *quadratic;

int xres, yres;

void init()
{
    // ... other code
    quadratic = gluNewQuadric();
    // ... other code

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
    
    glFrustum(-1.0, 1.0,
              -1.0, 1.0,
              1.0, 60.0);

    glMatrixMode(GL_MODELVIEW);
}

void drawIBar()
{
    /* Parameters for drawing the cylinders */
    float cyRad = 0.2, cyHeight = 1.0;
    int quadStacks = 4, quadSlices = 4;
    
    glPushMatrix();
    glColor3f(0, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 1, 0, 0);
    gluCylinder(quadratic, cyRad, cyRad, 2.0 * cyHeight, quadSlices, quadStacks);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(0, 1, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(1, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(1, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(0, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
}

float calcCatmullRomSpline(float p1, float p2, float p3, float p4, float u) {
    Matrix4f B = Eigen::Matrix4f::Zero();
    B <<  0.0,  1.0,  0.0,  0.0,
         -0.5,  0.0,  0.5,  0.0,
          1.0, -2.5,  2.0, -0.5,
         -0.5,  1.5, -1.5,  0.5;

    Vector4f u_vec(1.0, u, u*u, u*u*u);
    Vector4f p_vec(p1, p2, p3, p4);

    return u_vec.dot(B * p_vec);
}

void calcCatmullRomSpline(float *p1, float *p2, float *p3, float *p4, float u, float *result) {
    result[0] = calcCatmullRomSpline(p1[0], p2[0], p3[0], p4[0], u);
    result[1] = calcCatmullRomSpline(p1[1], p2[1], p3[1], p4[1], u);
    result[2] = calcCatmullRomSpline(p1[2], p2[2], p3[2], p4[2], u);
}

void calcCatmullRomSpline(MyQuaternion& p1, MyQuaternion& p2, MyQuaternion& p3, MyQuaternion& p4, float u, float *result) {
    result[0] = calcCatmullRomSpline(p1.v[0], p2.v[0], p3.v[0], p4.v[0], u);
    result[1] = calcCatmullRomSpline(p1.v[1], p2.v[1], p3.v[1], p4.v[1], u);
    result[2] = calcCatmullRomSpline(p1.v[2], p2.v[2], p3.v[2], p4.v[2], u);
    result[3] = calcCatmullRomSpline(p1.s, p2.s, p3.s, p4.s, u);
}

void computeTransformation(float *translation, float *scale, float *rotation) {
    int n_keyframes = script->keyframes->size();

    int kfi0_idx = -1;
    int kfi1_idx = -1;
    for (int i = 0; i < n_keyframes; i++) {
        KeyFrame *kf = script->keyframes->at(i);
        if (kf->frame <= current_frame) {
            kfi0_idx = i;
        }
        if (kf->frame > current_frame) {
            kfi1_idx = i;
            break;
        }
    }

    if (kfi0_idx == -1) {
        kfi0_idx = n_keyframes - 1; // is at end
    }
    if (kfi1_idx == -1) {
        kfi1_idx = 0; // next is at start
    }

    int kfi2_idx = (kfi1_idx + 1) % n_keyframes;
    int kfiMinus1_idx = (kfi0_idx + n_keyframes - 1) % n_keyframes;

    KeyFrame *kfi = script->keyframes->at(kfi0_idx);
    KeyFrame *kfi1 = script->keyframes->at(kfi1_idx);
    KeyFrame *kfi2 = script->keyframes->at(kfi2_idx);
    KeyFrame *kfiMinus1 = script->keyframes->at(kfiMinus1_idx);  

    int frame_range = (kfi1->frame - kfi->frame + script->n_frames) % script->n_frames;
    int offset = (current_frame - kfi->frame + script->n_frames) % script->n_frames;

    float u = 1.0 * offset / frame_range;

    float t_result[3];
    calcCatmullRomSpline(kfiMinus1->translation,
                         kfi->translation,
                         kfi1->translation,
                         kfi2->translation,
                         u,
                         t_result);
    float s_result[3];
    calcCatmullRomSpline(kfiMinus1->scale,
                         kfi->scale,
                         kfi1->scale,
                         kfi2->scale,
                         u,
                         s_result);

    MyQuaternion qiMinus1(kfiMinus1->rotation);
    MyQuaternion qi(kfi->rotation);
    MyQuaternion qi1(kfi1->rotation);
    MyQuaternion qi2(kfi2->rotation);
    float r_result[4];
    calcCatmullRomSpline(qiMinus1, qi, qi1, qi2, u, r_result);

    translation[0] = t_result[0];
    translation[1] = t_result[1];
    translation[2] = t_result[2];

    scale[0] = s_result[0];
    scale[1] = s_result[1];
    scale[2] = s_result[2];

    float norm = sqrt(r_result[0]*r_result[0] + r_result[1]*r_result[1] + r_result[2]*r_result[2] + r_result[3]*r_result[3]);
    rotation[0] = r_result[0] / norm;
    rotation[1] = r_result[1] / norm;
    rotation[2] = r_result[2] / norm;
    rotation[3] = r_result[3] / norm;
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Apply the rotation
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera
    glTranslatef(0.0, 0.0, -40.0);

    float translation[3], scale[3], rotation[4];
    computeTransformation(translation, scale, rotation);
    glTranslatef(translation[0], translation[1], translation[2]);

    float angle = 2.0 * acos(rotation[3]); // in radians
    float sinHalfAngle = sqrt(1.0 - rotation[3]*rotation[3]);
    if (fabs(sinHalfAngle) < 0.001) {
        sinHalfAngle = 1.0;
    }

    int x = rotation[0] / sinHalfAngle;
    int y = rotation[1] / sinHalfAngle;
    int z = rotation[2] / sinHalfAngle;
    glRotatef(angle * 180.0 / M_PI, x, y, z);

    glScalef(scale[0], scale[1], scale[2]);

    drawIBar();

    glutSwapBuffers();
}

void print_keyframes() {
    // Print the keyframes
    std::cout << "These are the keyframes" << std::endl;
    for (auto& keyframe: *script->keyframes) {
        float *t = keyframe->translation;
        float *s = keyframe->scale;
        float *r = keyframe->rotation;

        std::cout << "Frame " << keyframe->frame << std::endl;
        std::cout << "translation " << t[0] << " " << t[1] << " " << t[2] << std::endl;
        std::cout << "scale       " << s[0] << " " << s[1] << " " << s[2] << std::endl;
        std::cout << "rotation    " << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << std::endl;
    }
}

void read_test_script(std::string filename) {
    std::ifstream file(filename);
    size_t found = filename.find_last_of("/");
    std::string directory = filename.substr(0, found + 1);
    if (file.fail()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        exit(1);
    }

    std::string line;
    std::getline(file, line);
    int n_frames = atoi(line.c_str());

    script->keyframes = new std::vector<KeyFrame *>();
    script->n_frames = n_frames;

    while(std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        // Read in the keyframes for the object
        KeyFrame *keyframe = new KeyFrame;
        std::istringstream iss(line);
        std::string curr;

        iss >> curr; // "Frame"
        iss >> keyframe->frame;
        
        float x, y, z, theta;
        
        std::getline(file, line);
        iss = std::istringstream(line);
        iss >> curr; // "translation"
        iss >> x >> y >> z;
        keyframe->translation[0] = x;
        keyframe->translation[1] = y;
        keyframe->translation[2] = z;

        std::getline(file, line);
        iss = std::istringstream(line);
        iss >> curr; // "scale"
        iss >> x >> y >> z;
        keyframe->scale[0] = x;
        keyframe->scale[1] = y;
        keyframe->scale[2] = z;

        std::getline(file, line);
        iss = std::istringstream(line);
        iss >> curr; // "rotation"
        iss >> x >> y >> z >> theta;
        keyframe->rotation[0] = x;
        keyframe->rotation[1] = y;
        keyframe->rotation[2] = z;
        keyframe->rotation[3] = theta;

        script->keyframes->push_back(keyframe);
    }

    if (PRINT) {
        print_keyframes();
    }
}

void reshape(int width, int height) {
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    xres = viewport[2];
    yres = viewport[3];

    glViewport(0, 0, width, height);

    // May need to redo the frustrum calcs
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Set up the projection using glFrustum
    // near: 1.0
    // far: 60.0
    // left: -1.0
    // right: 1.0
    // bottom: -1.0
    // top: 1.0
    glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 60.0);

    glMatrixMode(GL_MODELVIEW);
    glutPostRedisplay();
}

void key_pressed(unsigned char key, int x, int y) {
    if (key == 'f') {
        std::cout << "Current Frame: " << current_frame << std::endl;
        return;
    }

    current_frame = (current_frame + 1) % script->n_frames;
    glutPostRedisplay();
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " [test.script]" << std::endl;
        exit(1);
    }

    read_test_script(argv[1]);

    /* 'glutInit' intializes the GLUT (Graphics Library Utility Toolkit) library.
     * This is necessary, since a lot of the functions we used above and below
     * are from the GLUT library.
     */
    glutInit(&argc, argv);
    /* The following line of code tells OpenGL that we need a double buffer,
     * a RGB pixel buffer, and a depth buffer.
     */
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    // glutInitWindowSize(xres, yres);
    glutInitWindowSize(500, 500);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("OpenGL Geometry Processing");

    init();

    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    xres = viewport[2];
    yres = viewport[3];

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    // Add keyboard to translate camera
    glutKeyboardFunc(key_pressed);

    glutMainLoop();
    
    return 0;
}