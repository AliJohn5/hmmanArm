#ifndef DATA_ALI
#define DATA_ALI

#include <vector>
#include <iostream>

typedef std::vector<float> Vec;
typedef std::vector<Vec> Mat;

void printMat(const Mat &m);

struct Position
{
    float x,y,z;
    float roll, pitch, yaw;
};

void printPos(const Position &pos);

#endif