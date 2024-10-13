#ifndef MATH_ALI
#define MATH_ALI

#include <vector>
#include "hummanArm/data.hpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include <queue>
#include <random>

Mat operator*(const Mat &a, const Mat &b);
bool operator==(const Mat &a, const Mat &b);

const float links[5] = {66.5, 335.8, 183, 55, 70};

#define HI(a) std::cout << "HI" << (a) << '\n'

Mat RXM(float ang_rad);
Mat RYM(float ang_rad);
Mat RZM(float ang_rad);
Mat TXM(float dist_in_mm);
Mat TYM(float dist_in_mm);
Mat TZM(float dist_in_mm);

Mat forward(Vec ang_rad);
Position getPosition(const Mat &tran_mat);
Mat getMat(const Position &pos);
float to_deg(float a);
void Best(
    Vec &old_ang,
    const Vec &new_ang,

    Position &cur_pos,
    const Position &tar_pos);

float dist(const Position &a, const Position &b);
float getdifRot(const Position &a, const Position &b);

struct Node
{
    Position p;
    Vec ang = {0, 0, 0, 0, 0};
    bool is_active = false;
};

class Robot
{
    float k = 25 * M_PI / 180;
    int min_x = -700;
    int min_y = -700;
    int min_z = -700;
    int max_x = 700;
    int max_y = 700;
    int max_z = 700;
    float getInc(
        Vec &current_ang,
        Position &current_pos,
        const Position &target_pos);

public:
    Robot();
    Mat fk(Vec ang);
    Vec ik(Mat mat);
};

#endif