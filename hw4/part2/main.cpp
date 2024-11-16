#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GLUT/glut.h>

#include <math.h>
#include <iostream>
#include <string>
#include <fstream>

#include "Eigen/Dense"

using namespace Eigen;

extern GLenum readpng(const char *filename);

// Variables for arcball rotation
Vector3f lastPos = Vector3f::Zero();
float currRot[16];
int isDragging = 0;
int lastX, lastY;

// Shaders
std::string vertProgFileName, fragProgFileName;
static GLuint vertexShader;
static GLuint fragmentShader;
static GLuint shaderProgram;
static GLenum colorMapTex, normalMapTex;
static GLint colorMapUniformPos, normalMapUniformPos;

void map_to_sphere(int x, int y, Vector3f* v) {
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    int xres = viewport[2], yres = viewport[3];
    // convert x and y to ndc
    double ndc_x = 1.0 * ((2.0 * x) / xres - 1.0);
    double ndc_y = -1 * ((2.0 * y) / yres - 1.0); // inverted yaxis to match pixels
    double dist2 = ndc_x * ndc_x + ndc_y * ndc_y;

    
    if (dist2 > 1.0) {
        float dist = sqrt(dist2);
        v->x() = ndc_x / dist;
        v->y() = ndc_y / dist;
        v->z() = 0.0f;
    } else {
        v->x() = ndc_x;
        v->y() = ndc_y;
        v->z() = sqrt(1.0 - dist2);
    }
}

// Function to initialize the rotation matrix
void initRotationMatrix() {
    for (int i = 0; i < 16; i++)
        currRot[i] = (i % 5 == 0) ? 1.0f : 0.0f; // Identity matrix
}

void mouseButton(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            isDragging = 1;
            lastX = x;
            lastY = y;
            map_to_sphere(x, y, &lastPos);
        } else {
            isDragging = 0;
        }
    }
}

void mouseMotion(int x, int y) {
    if (isDragging) {
        Vector3f currPos;
        map_to_sphere(x, y, &currPos);

        // Compute the rotation axis via cross product
        Vector3f axis = lastPos.cross(currPos);

        if (axis.norm() > 1e-6) {
            axis.normalize();

            float angle = acos(fmin(1.0f, lastPos.dot(currPos)));
            angle = angle * (180.0f / M_PI);

            // Update the current rotation matrix
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            glRotatef(angle, axis[0], axis[1], axis[2]);
            glMultMatrixf(currRot);
            glGetFloatv(GL_MODELVIEW_MATRIX, currRot);
            glPopMatrix();

            lastPos = currPos;

            glutPostRedisplay();
        }
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Apply the rotation
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef(0.0f, 0.0f, -5.0f);
    glMultMatrixf(currRot);

    // Set the color to white
    glColor3f(1.0f, 1.0f, 1.0f);

    // Draw a square centered at the origin
    glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-2.0f, -2.0f, 0.0f); // Bottom Left
        glTexCoord2f(1.0f, 0.0f); glVertex3f(2.0f, -2.0f, 0.0f); // Bottom Right
        glTexCoord2f(1.0f, 1.0f); glVertex3f(2.0f,  2.0f, 0.0f); // Top Right
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-2.0f,  2.0f, 0.0f); // Top Left
    glEnd();

    glutSwapBuffers();
}

void reshape(int width, int height) {
    // Adjust the viewport based on geometry changes
    glViewport(0, 0, width, height);
}

static void read_shaders() {
   std::string vertProgramSource, fragProgramSource;
   
   std::ifstream vertProgFile(vertProgFileName.c_str());
   if (! vertProgFile)
      std::cerr << "Error opening vertex shader program\n";
   std::ifstream fragProgFile(fragProgFileName.c_str());
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

   shaderProgram = glCreateProgram();

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

   glAttachShader(shaderProgram, vertShader);
   glAttachShader(shaderProgram, fragShader);
   glLinkProgram(shaderProgram);
   std::cerr << "Enabling fragment program: " << gluErrorString(glGetError()) << std::endl;
   glGetProgramInfoLog(shaderProgram, 1024, &blah, buf);
   std::cerr << buf;

   std::cerr << "Enabling program object" << std::endl;
   glUseProgram(shaderProgram);

   colorMapUniformPos = glGetUniformLocation(shaderProgram, "colorMap");
   normalMapUniformPos = glGetUniformLocation(shaderProgram, "normalMap");

   glActiveTexture(GL_TEXTURE0);
   glBindTexture(GL_TEXTURE_2D, colorMapTex);
   glUniform1i(colorMapUniformPos, 0);

   glActiveTexture(GL_TEXTURE1);
   glBindTexture(GL_TEXTURE_2D, normalMapTex);
   glUniform1i(normalMapUniformPos, 1);

    glUseProgram(shaderProgram);
}

void initOpenGL(const char *colorMapFileName, const char *normalMapFileName) {
    // Initialize rotation matrix
    initRotationMatrix();

    // Enable depth testing for proper occlusion
    glEnable(GL_DEPTH_TEST);

    // Set up the projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    int xres = viewport[2], yres = viewport[3];

    gluPerspective(60.0, (float)xres/yres, 1.0, 50.0);

    std::cerr << "Loading textures" << std::endl;
    if(!(colorMapTex = readpng(colorMapFileName)))
        exit(1);
    if(!(normalMapTex = readpng(normalMapFileName)))
        exit(1);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " [color_texture.png] [normal_map.png]" << std::endl;
        exit(1);
    }

    // Initialize GLUT
    glutInit(&argc, argv);
    // Set display mode
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    // Set window size (these values may change if screen is too small)
    glutInitWindowSize(800, 800);
    // Set window position
    glutInitWindowPosition(0, 0);
    // Create the window
    glutCreateWindow("GLSL Texture Renderer");

    // Initialize OpenGL settings
    initOpenGL(argv[1], argv[2]);

    // Read shaders
    vertProgFileName = "vertexProgram.glsl";
    fragProgFileName = "fragmentProgram.glsl";
    read_shaders();

    // Set the display callback
    glutDisplayFunc(display);
    // Set the reshape callback
    glutReshapeFunc(reshape);
    // Set the mouse callbacks
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);

    // Start the main loop
    glutMainLoop();
    return 0;
}