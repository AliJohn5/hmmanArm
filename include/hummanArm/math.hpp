#ifndef MATH_ALI
#define MATH_ALI

#include <vector>
#include "hummanArm/data.hpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include <queue>
#include <random>
#include "hummanArm/debug.hpp"

Mat operator*(const Mat &a, const Mat &b);
bool operator==(const Mat &a, const Mat &b);

const float links[5] = {66.5, 335.8, 183, 55, 70};

const Mat IDEN = {
    Vec({1, 0, 0, 0}),
    Vec({0, 1, 0, 0}),
    Vec({0, 0, 1, 0}),
    Vec({0, 0, 0, 1})

};

#define HI(a) std::cout << "HI" << (a) << '\n'

Mat RXM(float ang_rad);
Mat RYM(float ang_rad);
Mat RZM(float ang_rad);
Mat TXM(float dist_in_mm);
Mat TYM(float dist_in_mm);
Mat TZM(float dist_in_mm);

Mat forwardS(Ang ang_rad, int iter);
Mat forward(Ang ang_rad);

Position getPosition(const Mat &tran_mat);
Mat getMat(const Position &pos);
float to_deg(float a);

void Best(
    Ang &old_ang,
    const Ang &new_ang,

    Position &cur_pos,
    const Position &tar_pos);

float dist(const Position &a, const Position &b);
float dist(const Position &a, const Mat &b);
float dist(const Mat &a, const Mat &b);

struct Node
{
    Position p;
    Ang ang = {0, 0, 0, 0, 0};
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
    std::vector<Ang> angels;
    Mat _mat, _mat4, _mat2, _d, ans;
    float _r, _s;

    void recThata1(Ang &ang);
    void recThata2(Ang &ang);
    void recThata3(Ang &ang);
    void recThata4(Ang &ang);
    void recThata5(Ang &ang);

public:
    Robot();
    Mat fk(Ang ang);
    std::vector<Ang> ik(Mat mat);
    std::vector<Ang> ik(
        float x,
        float y,
        float z,
        std::vector<std::vector<float>> target_orintation,
        OrintationMode orintationMode);
};

#endif