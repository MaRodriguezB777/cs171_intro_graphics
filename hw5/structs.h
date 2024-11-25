#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>

struct Vec3f
{
	float x, y, z;
    Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3f() {}
};

struct Vertex
{
    float x, y, z;
    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}
    Vertex() {}
};

struct Face
{
    int idx1, idx2, idx3;
    Face(int idx1, int idx2, int idx3) : idx1(idx1), idx2(idx2), idx3(idx3) {}
    Face() {}
};

struct Mesh_Data
{
    std::vector<Vertex*> *vertices;
    std::vector<Face*> *faces;
};

#endif
