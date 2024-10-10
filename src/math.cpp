
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
    /*
     */
    return ans;
}

Position getPosition(const Mat &matrix)
{
    if (matrix.size() != 4 || matrix[0].size() != 4)
    {
        throw std::runtime_error("Matrix must be 4x4.");
    }

    Position pose;

    pose.x = matrix[0][3];
    pose.y = matrix[1][3];
    pose.z = matrix[2][3];

    Vec R1 = matrix[0];
    Vec R2 = matrix[1];
    Vec R3 = matrix[2];

    pose.pitch = atan2(-R3[0], sqrt(R1[0] * R1[0] + R2[0] * R2[0]));
    if (cos(pose.pitch) != 0)
    {
        pose.roll = atan2(R2[2], R1[2]);
        pose.yaw = atan2(R1[1], R1[0]);
    }
    else
    {
        pose.roll = 0;
        pose.yaw = atan2(-R2[0], R3[0]);
    }

    return pose;
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
