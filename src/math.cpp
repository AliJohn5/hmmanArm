
#include "hummanArm/math.hpp"

Mat RXM(float ang_rad)
{
    return Mat({{1, 0, 0, 0},
                {0, cos(ang_rad), -sin(ang_rad), 0},
                {0, sin(ang_rad), cos(ang_rad), 0},
                {0, 0, 0, 1}});
}

Mat RYM(float ang_rad)
{
    return Mat({{cos(ang_rad), 0, sin(ang_rad), 0},
                {0, 1, 0, 0},
                {-sin(ang_rad), 0, cos(ang_rad), 0},
                {0, 0, 0, 1}});
}

Mat RZM(float ang_rad)
{
    return Mat({{cos(ang_rad), -sin(ang_rad), 0, 0},
                {sin(ang_rad), cos(ang_rad), 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}});
}

Mat TXM(float dist_in_mm)
{

    return Mat(
        {{1, 0, 0, dist_in_mm},
         {0, 1, 0, 0},
         {0, 0, 1, 0},
         {0, 0, 0, 1}});
}

Mat TYM(float dist_in_mm)
{

    return Mat(
        {{1, 0, 0, 0},
         {0, 1, 0, dist_in_mm},
         {0, 0, 1, 0},
         {0, 0, 0, 1}});
}

Mat TZM(float dist_in_mm)
{

    return Mat(
        {{1, 0, 0, 0},
         {0, 1, 0, 0},
         {0, 0, 1, dist_in_mm},
         {0, 0, 0, 1}});
}

Mat forward(Vec ang_rad)
{
    Vec links = {66.5, 335.8, 183, 55, 70};
    Mat ans = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}

    };

    if (ang_rad.size() > 0)
    {
        ans = ans * RZM(ang_rad[0]) * TZM(links[0]);
    }

    if (ang_rad.size() > 1)
    {
        ans = ans * RXM(ang_rad[1]) * TXM(links[1]);
    }
    if (ang_rad.size() > 2)
    {
        ans = ans * RXM(ang_rad[2]) * TXM(links[2]);
    }

    if (ang_rad.size() > 3)
    {
        ans = ans * RZM(ang_rad[3]) * TZM(links[3]);
    }

    if (ang_rad.size() > 4)
    {
        ans = ans * RXM(ang_rad[4]) * TXM(links[4]);
    }

    return ans;
}

Position getPosition(const Mat &transform)
{
    Position pos;

    pos.x = transform[0][3];
    pos.y = transform[1][3];
    pos.z = transform[2][3];

    float R[3][3] = {
        {transform[0][0], transform[0][1], transform[0][2]},
        {transform[1][0], transform[1][1], transform[1][2]},
        {transform[2][0], transform[2][1], transform[2][2]}};

    pos.yaw = atan2(R[1][0], R[0][0]);
    pos.pitch = asin(-R[2][0]);
    pos.roll = atan2(R[2][1], R[2][2]);

    return pos;
}

Mat operator*(const Mat &a, const Mat &b)
{
    if (a[0].size() != b.size())
    {
        std::cerr << "### ERROR: col in first Mat != raw in second Mat\n";
        exit(-1);
    }

    Mat res(a.size(), Vec(b[0].size()));

    for (int i = 0; i < a.size(); ++i)
    {
        for (int j = 0; j < b[0].size(); ++j)
        {
            for (int k = 0; k < b.size(); ++k)
            {
                res[i][j] += a[i][k] * b[k][j];
            }
        }
    }

    return res;
}

bool operator==(const Mat &a, const Mat &b)
{
    if (a.size() != b.size())
        return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
        if (a[i].size() != b[i].size())
            return false;

        for (size_t j = 0; j < a[0].size(); ++j)
        {
            if (abs(a[i][j] - b[i][j]) > 0.0000001)
                return false;
        }
    }
    return true;
}
