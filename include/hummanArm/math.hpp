#ifndef MATH_ALI
#define MATH_ALI

#include <vector>
#include "hummanArm/data.hpp"
#include <math.h>
#include <iostream>



Mat operator*(const Mat &a, const Mat &b);
bool operator==(const Mat &a, const Mat &b);

Mat RXM(float ang_rad);
Mat RYM(float ang_rad);
Mat RZM(float ang_rad);
Mat TXM(float dist_in_mm);
Mat TYM(float dist_in_mm);
Mat TZM(float dist_in_mm);

Mat forward(Vec ang_rad);
Position getPosition(const Mat& tran_mat);

#endif