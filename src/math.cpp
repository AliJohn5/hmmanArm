
#include "hummanArm/math.hpp"

Mat RTXM(double ang_rad, double dist_in_mm)
{
    return {Vec({1, 0, 0, dist_in_mm}),
            Vec({0, cos(ang_rad), -sin(ang_rad), 0}),
            Vec({0, sin(ang_rad), cos(ang_rad), 0}),
            Vec({0, 0, 0, 1})};
}

Mat RTYM(double ang_rad, double dist_in_mm)
{
    return {
        Vec({cos(ang_rad), 0, sin(ang_rad), 0}),
        Vec({0, 1, 0, dist_in_mm}),
        Vec({-sin(ang_rad), 0, cos(ang_rad), 0}),
        Vec({0, 0, 0, 1})};
}

Mat RTZM(double ang_rad, double dist_in_mm)
{
    return {
        Vec({cos(ang_rad), -sin(ang_rad), 0, 0}),
        Vec({sin(ang_rad), cos(ang_rad), 0, 0}),
        Vec({0, 0, 1, dist_in_mm}),
        Vec({0, 0, 0, 1})};
}

Mat RXM(double ang_rad)
{
    return {Vec({1, 0, 0, 0}),
            Vec({0, cos(ang_rad), -sin(ang_rad), 0}),
            Vec({0, sin(ang_rad), cos(ang_rad), 0}),
            Vec({0, 0, 0, 1})};
}

Mat RYM(double ang_rad)
{
    return {
        Vec({cos(ang_rad), 0, sin(ang_rad), 0}),
        Vec({0, 1, 0, 0}),
        Vec({-sin(ang_rad), 0, cos(ang_rad), 0}),
        Vec({0, 0, 0, 1})};
}

Mat RZM(double ang_rad)
{
    return {
        Vec({cos(ang_rad), -sin(ang_rad), 0, 0}),
        Vec({sin(ang_rad), cos(ang_rad), 0, 0}),
        Vec({0, 0, 1, 0}),
        Vec({0, 0, 0, 1})};
}

Mat TXM(double dist_in_mm)
{

    return {Vec({1, 0, 0, dist_in_mm}),
            Vec({0, 1, 0, 0}),
            Vec({0, 0, 1, 0}),
            Vec({0, 0, 0, 1})};
}

Mat TYM(double dist_in_mm)
{

    return {
        Vec({1, 0, 0, 0}),
        Vec({0, 1, 0, dist_in_mm}),
        Vec({0, 0, 1, 0}),
        Vec({0, 0, 0, 1})};
}

Mat TZM(double dist_in_mm)
{

    return {Vec({1, 0, 0, 0}),
            Vec({0, 1, 0, 0}),
            Vec({0, 0, 1, dist_in_mm}),
            Vec({0, 0, 0, 1})};
}

Mat forward(Ang ang_rad)
{

    return IDEN *
           RZM(ang_rad[0]) * TZM(links[0]) *
           RYM(ang_rad[1]) * TZM(links[1]) *
           RYM(ang_rad[2]) * TZM(links[2]) *
           RZM(ang_rad[3]) * TZM(links[3]) *
           RYM(ang_rad[4]) * TZM(links[4]);
}

Mat forwardS(Ang ang_rad, int iter = 5)
{
    // Vec links = {66.5, 335.8, 183, 55, 70};

    Mat ans = {
        Vec({1, 0, 0, 0}),
        Vec({0, 1, 0, 0}),
        Vec({0, 0, 1, 0}),
        Vec({0, 0, 0, 1})

    };

    if (iter > 0)
    {
        ans = ans * RZM(ang_rad[0]) * TZM(links[0]);
    }

    if (iter > 1)
    {
        ans = ans * RYM(ang_rad[1]) * TZM(links[1]);
    }

    if (iter > 2)
    {
        ans = ans * RYM(ang_rad[2]) * TZM(links[2]);
    }

    if (iter > 3)
    {
        ans = ans * RZM(ang_rad[3]) * TZM(links[3]);
    }

    if (iter > 4)
    {
        ans = ans * RYM(ang_rad[4]) * TZM(links[4]);
    }

    return ans;
}

Position getPosition(const Mat &matrix)
{

    Position pose;

    pose.x = matrix[0][3];
    pose.y = matrix[1][3];
    pose.z = matrix[2][3];

    pose.pitch = atan2(-matrix[2][0], sqrt(matrix[0][0] * matrix[0][0] + matrix[1][0] * matrix[1][0]));
    if (cos(pose.pitch) != 0)
    {
        pose.roll = atan2(matrix[1][2], matrix[0][2]);
        pose.yaw = atan2(matrix[0][1], matrix[0][0]);
    }
    else
    {
        pose.roll = 0;
        pose.yaw = atan2(-matrix[1][0], matrix[2][0]);
    }

    return pose;
}

Mat getMat(const Position &pos)
{
    Mat ans = {
        Vec({1, 0, 0, pos.x}),
        Vec({0, 1, 0, pos.y}),
        Vec({0, 0, 1, pos.z}),
        Vec({0, 0, 0, 1})

    };

    return ans * (RZM(pos.yaw) * RYM(pos.pitch) * RXM(pos.roll));
}

double dist(const Position &a, const Position &b)
{
    return (a.x - b.x) * (a.x - b.x) +
           (a.y - b.y) * (a.y - b.y) +
           (a.z - b.z) * (a.z - b.z);
}

double dist(const Position &a, const Mat &b)
{
    return (a.x - b[0][3]) * (a.x - b[0][3]) +
           (a.y - b[1][3]) * (a.y - b[1][3]) +
           (a.z - b[2][3]) * (a.z - b[2][3]);
}

double dist(const Mat &a, const Mat &b)
{
    return (a[0][3] - b[0][3]) * (a[0][3] - b[0][3]) +
           (a[1][3] - b[1][3]) * (a[1][3] - b[1][3]) +
           (a[2][3] - b[2][3]) * (a[2][3] - b[2][3]);
}

double to_deg(double a)
{
    return a * 180 / M_PI;
}

void iterate(
    Position &target_pos,
    Position &starting_pos,

    Ang &starting_ang,
    double first_ind,
    double last_ind,
    double inc_ind)
{

    Ang ans = starting_ang;
    double d = dist(target_pos, starting_pos);

    for (double i1 = first_ind; i1 <= last_ind; i1 += inc_ind)
    {
        for (double i2 = first_ind; i2 <= last_ind; i2 += inc_ind)
        {
            for (double i3 = first_ind; i3 <= last_ind; i3 += inc_ind)
            {
                for (double i4 = first_ind; i4 <= last_ind; i4 += inc_ind)
                {
                    for (double i5 = first_ind; i5 <= last_ind; i5 += inc_ind)
                    {
                        Ang new_ang = starting_ang;
                        new_ang[0] += i1;
                        new_ang[1] += i2;
                        new_ang[2] += i3;
                        new_ang[3] += i4;
                        new_ang[4] += i5;

                        Position p = getPosition(forward(new_ang));

                        if (dist(p, target_pos) < d)
                        {
                            d = dist(p, target_pos);
                            ans = new_ang;
                        }
                    }
                }
            }
        }
    }

    starting_ang = ans;
}

void iterates(
    Position &target_pos,
    Position &starting_pos,

    Ang &starting_ang,
    double first_ind,
    double last_ind,
    double inc_ind)
{

    Ang ans = starting_ang;
    double d = dist(target_pos, starting_pos);

    // for (double i4 = first_ind; i4 <= last_ind; i4 += inc_ind)
    //{
    for (double i5 = first_ind; i5 <= last_ind; i5 += inc_ind)
    {
        Ang new_ang = starting_ang;
        // new_ang[3] += i4;
        new_ang[4] += i5;

        Position p = getPosition(forward(new_ang));

        if (dist(p, target_pos) < d)
        {
            d = dist(p, target_pos);
            ans = new_ang;
        }

        if (d < 1 || d < inc_ind)
        {
            // std::cout << to_deg(i5) << '\n';
            starting_ang = ans;
            return;
        }
    }
    // }

    starting_ang = ans;
}

void binary_search(
    Position &target_pos,
    Position &starting_pos,

    Ang &starting_ang)
{

    Ang ans = starting_ang;
    double d = dist(target_pos, starting_pos);
    double inc = M_PI;
    Ang l_ang, r_ang;
    Mat pl, pr;
    double dl, dr;

    while (d > 0.5 && abs(inc) > 0.002)
    {
        l_ang[4] = ans[4] + inc;
        r_ang[4] = ans[4] - inc;

        pl = forward(l_ang);
        pr = forward(r_ang);

        dl = dist(target_pos, pl);
        dr = dist(target_pos, pr);

        if (dl < dr)
        {
            if (d > dl)
            {
                d = dl;
                ans = l_ang;
                inc *= 0.75;
            }
            else
                inc /= 2;
        }
        else
        {
            if (d > dr)
            {
                d = dr;
                ans = r_ang;
                inc *= 0.75;
            }
            else
                inc /= 2;
        }
    }
    // }

    // std::cout << "iter " << iter << '\n';
    starting_ang = ans;
}

void Best(
    Ang &old_ang,
    const Ang &new_ang,

    Position &cur_pos,
    const Position &tar_pos)
{
    Position forw = getPosition(forward(new_ang));
    if (dist(forw, tar_pos) < dist(cur_pos, tar_pos))
    {
        cur_pos = forw;
        old_ang = new_ang;
    }
    return;
}

Mat operator*(const Mat &a, const Mat &b)
{
    Mat res = {
        Vec({0, 0, 0, 0}),
        Vec({0, 0, 0, 0}),
        Vec({0, 0, 0, 0}),
        Vec({0, 0, 0, 0}),
    };

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

void generateForwardFunction()
{
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

Robot::Robot()
{
}

Mat Robot::fk(Ang ang)
{
    return forward(ang);
}

void Robot::recThata1(Ang &ang)
{
    _mat4 = _mat * TZM(-links[4]);
    ang[0] = atan2(_mat4[1][3], _mat4[0][3]);

    if (std::isnan(ang[0]))
    {
        ang[0] = 0;
        recThata3(ang);

        ang[0] = M_PI;
        recThata3(ang);
    }
    else
    {
        recThata3(ang);
        ang[0] *= -1;
        recThata3(ang);
    }
}

void Robot::recThata3(Ang &ang)
{

    _r = sqrt(_mat4[0][3] * _mat4[0][3] + _mat4[1][3] * _mat4[1][3]);
    _s = _mat4[2][3] - links[0];

    ang[2] = acos(
        (_r * _r + _s * _s - (links[2] + links[3]) * (links[2] + links[3]) - links[1] * links[1]) /
        (2 * (links[2] + links[3]) * links[1]));

    if (std::isnan(ang[2]))
    {
        ang[2] = 0;
        recThata2(ang);

        ang[2] = M_PI;
        recThata2(ang);
    }

    else
    {
        recThata2(ang);
        ang[2] *= -1;
        recThata2(ang);
    }
}

void Robot::recThata2(Ang &ang)
{

    ang[1] = M_PI_2 - (asin(
                           (
                               (links[2] + links[3]) * sin(M_PI - ang[2])) /
                           sqrt(_s * _s + _r * _r)) +
                       atan2(_s, _r));

    if (std::isnan(ang[1]))
    {
        ang[1] = 0;
        recThata4(ang);

        ang[1] = M_PI;
        recThata4(ang);
    }

    else
    {
        recThata4(ang);
        ang[1] *= -1;
        recThata4(ang);
    }
}

void Robot::recThata4(Ang &ang)
{

    _mat2 = forwardS({ang[0], ang[1], ang[2], 0, 0}, 3);

    std::swap(_mat2[1][0], _mat2[0][1]);
    std::swap(_mat2[2][0], _mat2[0][2]);
    std::swap(_mat2[1][2], _mat2[2][1]);

    _d = _mat2 * _mat4;

    ang[3] = atan2(_d[1][2], _d[0][2]);

    if (std::isnan(ang[3]))
    {
        ang[3] = 0;
        recThata5(ang);

        ang[3] = M_PI;
        recThata5(ang);
    }

    else
    {
        recThata5(ang);
        ang[3] *= -1;
        recThata5(ang);
    }
}

void Robot::recThata5(Ang &ang)
{
    ang[4] = acos(_d[2][2]);

    if (std::isnan(ang[4]))
    {
        ang[4] = 0;
    }

    ans = forward(ang);

    if (dist(ans, _mat) < 1)
    {
        angels.push_back(ang);
    }
}

std::vector<Ang> Robot::ik(Mat mat)
{
    _mat = mat;
    angels.clear();
    Ang ang;
    recThata1(ang);

    if (angels.size() == 0)
    {
        angels.push_back(Ang({0, 0, 0, 0, 0}));
    }
    return angels;
}

std::vector<Ang> Robot::ik(
    double x,
    double y,
    double z,
    std::vector<std::vector<double>> target_orintation,
    OrintationMode orintationMode)
{
    Mat mat;

    switch (orintationMode)
    {
    case OrintationMode::x:
        for (size_t i = 0; i < 3; i++)
        {
            mat[0][i] = target_orintation[0][i];
        }
        break;

    case OrintationMode::y:
        for (size_t i = 0; i < 3; i++)
        {
            mat[1][i] = target_orintation[0][i];
        }
        break;

    case OrintationMode::z:
        for (size_t i = 0; i < 3; i++)
        {
            mat[2][i] = target_orintation[0][i];
        }
        break;

    case OrintationMode::all:
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                mat[i][j] = target_orintation[i][j];
            }
        }
        break;

    default:
        break;
    }

    mat[0][3] = x;
    mat[1][3] = y;
    mat[2][3] = z;

    return ik(mat);
}