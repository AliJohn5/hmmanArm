#ifndef DATA_ALI
#define DATA_ALI

#include <vector>
#include <iostream>
#include <array>

typedef std::array<float, 5> Ang;

typedef std::array<float, 4> Vec;
typedef std::array<Vec, 4> Mat;

void printMat(const Mat &m);

struct Position
{
    float x, y, z;
    float roll, pitch, yaw;
};

void printPos(const Position &pos);

enum OrintationMode
{
    x,
    y,
    z,
    all
};

#endif