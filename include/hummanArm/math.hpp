#ifndef MATH_ALI
#define MATH_ALI

#include <vector>
#include "hummanArm/data.hpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include "hummanArm/debug.hpp"
#include <map>
#include <set>


#define DEBUGER_ALI

Mat operator*(const Mat &a, const Mat &b);
bool operator==(const Mat &a, const Mat &b);

const double links[5] = {66.5, 335.8, 183, 55, 70};

const Mat IDEN = {
    Vec({1, 0, 0, 0}),
    Vec({0, 1, 0, 0}),
    Vec({0, 0, 1, 0}),
    Vec({0, 0, 0, 1})

};

#define HI(a) std::cout << "HI" << (a) << '\n'

Mat RXM(double ang_rad);
Mat RYM(double ang_rad);
Mat RZM(double ang_rad);
Mat TXM(double dist_in_mm);
Mat TYM(double dist_in_mm);
Mat TZM(double dist_in_mm);

Mat RTXM(double ang_rad, double dist_in_mm);
Mat RTYM(double ang_rad, double dist_in_mm);
Mat RTZM(double ang_rad, double dist_in_mm);

Mat forwardS(Ang ang_rad, int iter);
Mat forward(Ang ang_rad);

Position getPosition(const Mat &tran_mat);
Mat getMat(const Position &pos);
double to_deg(double a);

void Best(
    Ang &old_ang,
    const Ang &new_ang,

    Position &cur_pos,
    const Position &tar_pos);

double dist(const Position &a, const Position &b);
double dist(const Position &a, const Mat &b);
double dist(const Mat &a, const Mat &b);
double distWithDir(const Mat &a, const Mat &b);

struct Node
{
    Position p;
    Ang ang = {0, 0, 0, 0, 0};
    bool is_active = false;
};

class Robot
{
    double k = 25 * M_PI / 180;
    int min_x = -700;
    int min_y = -700;
    int min_z = -700;
    int max_x = 700;
    int max_y = 700;
    int max_z = 700;
    std::vector<Ang> angels;
    std::vector<Mat> angelsForward;

    Mat _mat, _mat4, _mat2, _d, ans;
    double _r, _s;

    void recThata1(Ang &ang,std::string);
    void recThata2(Ang &ang,std::string);
    void recThata3(Ang &ang,std::string);
    void recThata4(Ang &ang,std::string);
    void recThata5(Ang &ang,std::string);

public:
    Robot();
    std::set<std::string> strings;
    Mat fk(Ang ang);
    std::vector<Ang> ik(Mat mat);
    std::vector<Ang> ik(
        double x,
        double y,
        double z,
        std::vector<std::vector<double>> target_orintation,
        OrintationMode orintationMode);
    Ang IKNearAng(
        double x,
        double y,
        double z,
        std::vector<std::vector<double>> target_orintation,
        OrintationMode orintationMode);
};

Mat forwardUsingEquations(Ang ang_rad);
void forwardUsingEquations3(Ang ang_rad, Mat &ans);
std::vector<Ang> inverseUsingEquations(Mat mat);

void testAnyThingHere();
#endif