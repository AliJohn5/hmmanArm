
#include "hummanArm/math.hpp"
#include "hummanArm/sympole.hpp"

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

double distWithDir(const Mat &a, const Mat &b)
{
    double ans = 0;
    for (size_t i = 0; i < 4; ++i)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            ans += (a[i][j] - b[i][j]) * (a[i][j] - b[i][j]);
        }
    }
    return ans;
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
    return forwardUsingEquations(ang);
}

void forwardUsingEquations3(Ang ang_rad, Mat &ans)
{
    double sinV[5];
    double cosV[5];
    for (int i = 0; i < 5; ++i)
    {
        sinV[i] = sin(ang_rad[i]);
        cosV[i] = cos(ang_rad[i]);
    }
    ans[0][0] = (((cosV[0] * cosV[1]) * cosV[2]) + ((cosV[0] * sinV[1]) * -sinV[2]));
    ans[0][1] = -sinV[0];
    ans[0][2] = (((cosV[0] * cosV[1]) * sinV[2]) + ((cosV[0] * sinV[1]) * cosV[2]));
    ans[0][3] = (((((cosV[0] * cosV[1]) * sinV[2]) + ((cosV[0] * sinV[1]) * cosV[2])) * 183) + ((cosV[0] * sinV[1]) * 335.8));
    ans[1][0] = (((sinV[0] * cosV[1]) * cosV[2]) + ((sinV[0] * sinV[1]) * -sinV[2]));
    ans[1][1] = cosV[0];
    ans[1][2] = (((sinV[0] * cosV[1]) * sinV[2]) + ((sinV[0] * sinV[1]) * cosV[2]));
    ans[1][3] = (((((sinV[0] * cosV[1]) * sinV[2]) + ((sinV[0] * sinV[1]) * cosV[2])) * 183) + ((sinV[0] * sinV[1]) * 335.8));
    ans[2][0] = ((-sinV[1] * cosV[2]) + (cosV[1] * -sinV[2]));
    ans[2][1] = 0;
    ans[2][2] = ((-sinV[1] * sinV[2]) + (cosV[1] * cosV[2]));
    ans[2][3] = ((((-sinV[1] * sinV[2]) + (cosV[1] * cosV[2])) * 183) + ((cosV[1] * 335.8) + 66.5));
    ans[3][0] = 0;
    ans[3][1] = 0;
    ans[3][2] = 0;
    ans[3][3] = 1;
}

void Robot::recThata1(Ang &ang, std::string s)
{
    _mat4 = _mat * TZM(-links[4]);
    ang[0] = atan2(_mat4[1][3], _mat4[0][3]);
    recThata3(ang, s + "2");

    /**
    if (std::isnan(ang[0]))
    {
        ang[0] = 0;
        recThata3(ang, s + "0");

        ang[0] = M_PI;
        recThata3(ang, s + "1");
    }
    else
    {
        recThata3(ang, s + "2");
        ang[0] *= -1;
        recThata3(ang, s + "3");
    }
    */
}

void Robot::recThata3(Ang &ang, std::string s)
{

    _r = sqrt(_mat4[0][3] * _mat4[0][3] + _mat4[1][3] * _mat4[1][3]);
    _s = _mat4[2][3] - links[0];

    ang[2] = acos(
        (_r * _r + _s * _s - (links[2] + links[3]) * (links[2] + links[3]) - links[1] * links[1]) /
        (2 * (links[2] + links[3]) * links[1]));

    recThata2(ang, s + "2");
    ang[2] *= -1;
    recThata2(ang, s + "3");
    /**
        if (std::isnan(ang[2]))
        {
            ang[2] = 0;
            recThata2(ang, s + "0");

            ang[2] = M_PI;
            recThata2(ang, s + "1");
        }

        else
        {
            recThata2(ang, s + "2");
            ang[2] *= -1;
            recThata2(ang, s + "3");
        }
        */
}

void Robot::recThata2(Ang &ang, std::string s)
{

    ang[1] = M_PI_2 - (asin(
                           (
                               (links[2] + links[3]) * sin(M_PI - ang[2])) /
                           sqrt(_s * _s + _r * _r)) +
                       atan2(_s, _r));

    recThata4(ang, s + "2");

    /**
        if (std::isnan(ang[1]))
        {
            ang[1] = 0;
            recThata4(ang, s + "0");

            ang[1] = M_PI;
            recThata4(ang, s + "1");
        }

        else
        {
            recThata4(ang, s + "2");
            ang[1] *= -1;
            recThata4(ang, s + "3");
        }
    */
}

void Robot::recThata4(Ang &ang, std::string s)
{

    forwardUsingEquations3({ang[0], ang[1], ang[2], 0, 0}, _mat2);

    std::swap(_mat2[1][0], _mat2[0][1]);
    std::swap(_mat2[2][0], _mat2[0][2]);
    std::swap(_mat2[1][2], _mat2[2][1]);

    _d = _mat2 * _mat4;

    ang[3] = atan2(_d[1][2], _d[0][2]);
    recThata5(ang, s + "2");
    /**
        if (std::isnan(ang[3]))
        {
            ang[3] = 0;
            recThata5(ang, s + "0");

            ang[3] = M_PI;
            recThata5(ang, s + "1");
        }

        else
        {
            recThata5(ang, s + "2");
            ang[3] *= -1;
            recThata5(ang, s + "3");
        }
        */
}

void Robot::recThata5(Ang &ang, std::string s)
{
    ang[4] = acos(_d[2][2]);

    if (std::isnan(ang[4]))
    {
        ang[4] = 0;
    }

    ans = forwardUsingEquations(ang);

    if (dist(ans, _mat) < 1)
    {
        angels.push_back(ang);
        angelsForward.push_back(ans);
        strings.insert(s + "0");
    }
}

std::vector<Ang> Robot::ik(Mat mat)
{
#ifndef DEBUGER_ALI
    return inverseUsingEquations(mat);
#endif
    _mat = mat;
    angels.clear();
    angelsForward.clear();
    strings.clear();
    Ang ang;
    recThata1(ang, "");

    if (angels.size() == 0)
    {
        angels.push_back(Ang({0, 0, 0, 0, 0}));
        angelsForward.push_back(forward(Ang({0, 0, 0, 0, 0})));
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

    return inverseUsingEquations(mat);
}

Ang Robot::IKNearAng(
    double x,
    double y,
    double z,
    std::vector<std::vector<double>> target_orintation,
    OrintationMode orintationMode)
{
    // TODO: this need some works
    ik(x, y, z, target_orintation, orintationMode);

    std::map<double, Ang> m;
    double temp = 1000;

    for (int i = 0; i < angelsForward.size(); ++i)
    {
        temp = distWithDir(_mat, angelsForward[i]);
        m[temp] = angels[i];
    }

    return m.begin()->second;
}

Mat forwardUsingEquations(Ang ang_rad)
{
    Mat ans;
    double sinV[5];
    double cosV[5];
    for (int i = 0; i < 5; ++i)
    {
        sinV[i] = sin(ang_rad[i]);
        cosV[i] = cos(ang_rad[i]);
    }
    ans[0][0] = (((((((cosV[0] * cosV[1]) * cosV[2]) + ((cosV[0] * sinV[1]) * -sinV[2])) * cosV[3]) + (-sinV[0] * sinV[3])) * cosV[4]) + ((((cosV[0] * cosV[1]) * sinV[2]) + ((cosV[0] * sinV[1]) * cosV[2])) * -sinV[4]));
    ans[0][1] = (((((cosV[0] * cosV[1]) * cosV[2]) + ((cosV[0] * sinV[1]) * -sinV[2])) * -sinV[3]) + (-sinV[0] * cosV[3]));
    ans[0][2] = (((((((cosV[0] * cosV[1]) * cosV[2]) + ((cosV[0] * sinV[1]) * -sinV[2])) * cosV[3]) + (-sinV[0] * sinV[3])) * sinV[4]) + ((((cosV[0] * cosV[1]) * sinV[2]) + ((cosV[0] * sinV[1]) * cosV[2])) * cosV[4]));
    ans[0][3] = (((((((((cosV[0] * cosV[1]) * cosV[2]) + ((cosV[0] * sinV[1]) * -sinV[2])) * cosV[3]) + (-sinV[0] * sinV[3])) * sinV[4]) + ((((cosV[0] * cosV[1]) * sinV[2]) + ((cosV[0] * sinV[1]) * cosV[2])) * cosV[4])) * 70) + (((((cosV[0] * cosV[1]) * sinV[2]) + ((cosV[0] * sinV[1]) * cosV[2])) * 55) + (((((cosV[0] * cosV[1]) * sinV[2]) + ((cosV[0] * sinV[1]) * cosV[2])) * 183) + ((cosV[0] * sinV[1]) * 335.8))));
    ans[1][0] = (((((((sinV[0] * cosV[1]) * cosV[2]) + ((sinV[0] * sinV[1]) * -sinV[2])) * cosV[3]) + (cosV[0] * sinV[3])) * cosV[4]) + ((((sinV[0] * cosV[1]) * sinV[2]) + ((sinV[0] * sinV[1]) * cosV[2])) * -sinV[4]));
    ans[1][1] = (((((sinV[0] * cosV[1]) * cosV[2]) + ((sinV[0] * sinV[1]) * -sinV[2])) * -sinV[3]) + (cosV[0] * cosV[3]));
    ans[1][2] = (((((((sinV[0] * cosV[1]) * cosV[2]) + ((sinV[0] * sinV[1]) * -sinV[2])) * cosV[3]) + (cosV[0] * sinV[3])) * sinV[4]) + ((((sinV[0] * cosV[1]) * sinV[2]) + ((sinV[0] * sinV[1]) * cosV[2])) * cosV[4]));
    ans[1][3] = (((((((((sinV[0] * cosV[1]) * cosV[2]) + ((sinV[0] * sinV[1]) * -sinV[2])) * cosV[3]) + (cosV[0] * sinV[3])) * sinV[4]) + ((((sinV[0] * cosV[1]) * sinV[2]) + ((sinV[0] * sinV[1]) * cosV[2])) * cosV[4])) * 70) + (((((sinV[0] * cosV[1]) * sinV[2]) + ((sinV[0] * sinV[1]) * cosV[2])) * 55) + (((((sinV[0] * cosV[1]) * sinV[2]) + ((sinV[0] * sinV[1]) * cosV[2])) * 183) + ((sinV[0] * sinV[1]) * 335.8))));
    ans[2][0] = (((((-sinV[1] * cosV[2]) + (cosV[1] * -sinV[2])) * cosV[3]) * cosV[4]) + (((-sinV[1] * sinV[2]) + (cosV[1] * cosV[2])) * -sinV[4]));
    ans[2][1] = (((-sinV[1] * cosV[2]) + (cosV[1] * -sinV[2])) * -sinV[3]);
    ans[2][2] = (((((-sinV[1] * cosV[2]) + (cosV[1] * -sinV[2])) * cosV[3]) * sinV[4]) + (((-sinV[1] * sinV[2]) + (cosV[1] * cosV[2])) * cosV[4]));
    ans[2][3] = (((((((-sinV[1] * cosV[2]) + (cosV[1] * -sinV[2])) * cosV[3]) * sinV[4]) + (((-sinV[1] * sinV[2]) + (cosV[1] * cosV[2])) * cosV[4])) * 70) + ((((-sinV[1] * sinV[2]) + (cosV[1] * cosV[2])) * 55) + ((((-sinV[1] * sinV[2]) + (cosV[1] * cosV[2])) * 183) + ((cosV[1] * 335.8) + 66.5))));
    ans[3][0] = 0;
    ans[3][1] = 0;
    ans[3][2] = 0;
    ans[3][3] = 1;
    return ans;
}

std::vector<Ang> inverseUsingEquations(Mat mat)
{
    std::vector<Ang> ans(2);
    ans[0][0] = ans[1][0] = atan2(((mat[1][2] * -70) + mat[1][3]), ((mat[0][2] * -70) + mat[0][3]));

    ans[0][2] = acos(((((sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3])))) * sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3]))))) + ((((mat[2][2] * -70) + mat[2][3]) - 66.5) * (((mat[2][2] * -70) + mat[2][3]) - 66.5))) - (238.000000 * 238.000000) - 112761.640000) / (2 * 238.000000 * 335.800000)));

    ans[1][2] = -acos(((((sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3])))) * sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3]))))) + ((((mat[2][2] * -70) + mat[2][3]) - 66.5) * (((mat[2][2] * -70) + mat[2][3]) - 66.5))) - (238.000000 * 238.000000) - 112761.640000) / (2 * 238.000000 * 335.800000)));

    ans[0][1] = M_PI_2 - (asin((238.000000 * sin(M_PI - ans[0][2])) / sqrt(((((mat[2][2] * -70) + mat[2][3]) - 66.5) * (((mat[2][2] * -70) + mat[2][3]) - 66.5)) + (sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3])))) * sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3]))))))) + atan2((((mat[2][2] * -70) + mat[2][3]) - 66.5), sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3]))))));

    ans[1][1] = M_PI_2 - (asin((238.000000 * sin(M_PI - ans[1][2])) / sqrt(((((mat[2][2] * -70) + mat[2][3]) - 66.5) * (((mat[2][2] * -70) + mat[2][3]) - 66.5)) + (sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3])))) * sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3]))))))) + atan2((((mat[2][2] * -70) + mat[2][3]) - 66.5), sqrt(((((mat[0][2] * -70) + mat[0][3]) * ((mat[0][2] * -70) + mat[0][3])) + (((mat[1][2] * -70) + mat[1][3]) * ((mat[1][2] * -70) + mat[1][3]))))));

    ans[0][3] = atan2((((-sin(ans[0][0]) * mat[0][2]) + (cos(ans[0][0]) * mat[1][2])) + ((((((sin(ans[0][0]) * cos(ans[0][1])) * sin(ans[0][2])) + ((sin(ans[0][0]) * sin(ans[0][1])) * cos(ans[0][2]))) * 183) + ((sin(ans[0][0]) * sin(ans[0][1])) * 335.8)) * mat[3][2])), (((((((cos(ans[0][0]) * cos(ans[0][1])) * cos(ans[0][2])) + ((cos(ans[0][0]) * sin(ans[0][1])) * -sin(ans[0][2]))) * mat[0][2]) + ((((sin(ans[0][0]) * cos(ans[0][1])) * cos(ans[0][2])) + ((sin(ans[0][0]) * sin(ans[0][1])) * -sin(ans[0][2]))) * mat[1][2])) + (((-sin(ans[0][1]) * cos(ans[0][2])) + (cos(ans[0][1]) * -sin(ans[0][2]))) * mat[2][2])) + ((((((cos(ans[0][0]) * cos(ans[0][1])) * sin(ans[0][2])) + ((cos(ans[0][0]) * sin(ans[0][1])) * cos(ans[0][2]))) * 183) + ((cos(ans[0][0]) * sin(ans[0][1])) * 335.8)) * mat[3][2])));

    ans[1][3] = atan2((((-sin(ans[1][0]) * mat[0][2]) + (cos(ans[1][0]) * mat[1][2])) + ((((((sin(ans[1][0]) * cos(ans[1][1])) * sin(ans[1][2])) + ((sin(ans[1][0]) * sin(ans[1][1])) * cos(ans[1][2]))) * 183) + ((sin(ans[1][0]) * sin(ans[1][1])) * 335.8)) * mat[3][2])), (((((((cos(ans[1][0]) * cos(ans[1][1])) * cos(ans[1][2])) + ((cos(ans[1][0]) * sin(ans[1][1])) * -sin(ans[1][2]))) * mat[0][2]) + ((((sin(ans[1][0]) * cos(ans[1][1])) * cos(ans[1][2])) + ((sin(ans[1][0]) * sin(ans[1][1])) * -sin(ans[1][2]))) * mat[1][2])) + (((-sin(ans[1][1]) * cos(ans[1][2])) + (cos(ans[1][1]) * -sin(ans[1][2]))) * mat[2][2])) + ((((((cos(ans[1][0]) * cos(ans[1][1])) * sin(ans[1][2])) + ((cos(ans[1][0]) * sin(ans[1][1])) * cos(ans[1][2]))) * 183) + ((cos(ans[1][0]) * sin(ans[1][1])) * 335.8)) * mat[3][2])));

    ans[0][4] = acos((((((((cos(ans[0][0]) * cos(ans[0][1])) * sin(ans[0][2])) + ((cos(ans[0][0]) * sin(ans[0][1])) * cos(ans[0][2]))) * mat[0][2]) + ((((sin(ans[0][0]) * cos(ans[0][1])) * sin(ans[0][2])) + ((sin(ans[0][0]) * sin(ans[0][1])) * cos(ans[0][2]))) * mat[1][2])) + (((-sin(ans[0][1]) * sin(ans[0][2])) + (cos(ans[0][1]) * cos(ans[0][2]))) * mat[2][2])) + (((((-sin(ans[0][1]) * sin(ans[0][2])) + (cos(ans[0][1]) * cos(ans[0][2]))) * 183) + ((cos(ans[0][1]) * 335.8) + 66.5)) * mat[3][2])));

    ans[1][4] = acos((((((((cos(ans[1][0]) * cos(ans[1][1])) * sin(ans[1][2])) + ((cos(ans[1][0]) * sin(ans[1][1])) * cos(ans[1][2]))) * mat[0][2]) + ((((sin(ans[1][0]) * cos(ans[1][1])) * sin(ans[1][2])) + ((sin(ans[1][0]) * sin(ans[1][1])) * cos(ans[1][2]))) * mat[1][2])) + (((-sin(ans[1][1]) * sin(ans[1][2])) + (cos(ans[1][1]) * cos(ans[1][2]))) * mat[2][2])) + (((((-sin(ans[1][1]) * sin(ans[1][2])) + (cos(ans[1][1]) * cos(ans[1][2]))) * 183) + ((cos(ans[1][1]) * 335.8) + 66.5)) * mat[3][2])));

    return ans;
}

void testAnyThingHere()
{
    MatS mat = {
        VecS({"mat[0][0]", "mat[0][1]", "mat[0][2]", "mat[0][3]"}),
        VecS({"mat[1][0]", "mat[1][1]", "mat[1][2]", "mat[1][3]"}),
        VecS({"mat[2][0]", "mat[2][1]", "mat[2][2]", "mat[2][3]"}),
        VecS({"mat[3][0]", "mat[3][1]", "mat[3][2]", "mat[3][3]"})};

    MatS mat4 = mat * TZMS("-" + linksS[4]);
    std::string s = "std::vector<Ang> inverseUsingEquations(Mat mat)\n{\n\tstd::vector<Ang> ans(2);\n";

    s += "\tans[0][0] = ans[1][0] = " + atan2S(mat4[1][3], mat4[0][3]) + ";\n\n";

    std::string h1 = prodTwoString(mat4[0][3], mat4[0][3]);
    std::string h2 = prodTwoString(mat4[1][3], mat4[1][3]);
    std::string h3 = sumTwoString(h1, h2);
    std::string _r = sqrtS(h3);
    std::string _s = "(" + mat4[2][3] + "-" + linksS[0] + ")";

    std::string a11 = std::to_string(links[2] + links[3]);
    std::string a21 = std::to_string(links[1] * links[1]);
    std::string a31 = std::to_string(links[1]);
    std::string a41 = prodTwoString(_r, _r);
    std::string a51 = prodTwoString(_s, _s);
    std::string a61 = prodTwoString(a11, a11);
    std::string a71 = sumTwoString(a41, a51);
    std::string a81 = "(" + a71 + "-" + a61 + "-" + a21 + ")";

    std::string a91 = "(2*" + a11 + "*" + a31 + ")";
    std::string a110 = "(" + a81 + "/" + a91 + ")";

    s += "\tans[0][2] = " + acosS(a110) + ";\n\n";
    s += "\tans[1][2] = -" + acosS(a110) + ";\n\n";

    std::string b01 = sinS("M_PI-ans[0][2]");
    std::string b11 = sinS("M_PI-ans[1][2]");
    std::string b03 = atan2S(_s, _r);
    std::string b04 = sqrtS(a51 + "+" + a41);

    std::string b05 = "(" + asinS("(" + a11 + "*" + b01 + ")/" + b04) + "+" + b03 + ")";
    std::string b06 = "(" + asinS("(" + a11 + "*" + b11 + ")/" + b04) + "+" + b03 + ")";

    s += "\tans[0][1] = M_PI_2-" + b05 + ";\n\n";

    s += "\tans[1][1] = M_PI_2-" + b06 + ";\n\n";

    MatS _mat20 = generateForwardEquations({"ans[0][0]", "ans[0][1]", "ans[0][2]"});
    MatS _mat21 = generateForwardEquations({"ans[1][0]", "ans[1][1]", "ans[1][2]"});

    std::swap(_mat20[1][0], _mat20[0][1]);
    std::swap(_mat20[2][0], _mat20[0][2]);
    std::swap(_mat20[1][2], _mat20[2][1]);

    std::swap(_mat21[1][0], _mat21[0][1]);
    std::swap(_mat21[2][0], _mat21[0][2]);
    std::swap(_mat21[1][2], _mat21[2][1]);

    MatS _d1 = _mat20 * mat4;
    MatS _d2 = _mat21 * mat4;

    s += "\tans[0][3]=" + atan2S(_d1[1][2], _d1[0][2]) + ";\n\n";
    s += "\tans[1][3]=" + atan2S(_d2[1][2], _d2[0][2]) + ";\n\n";

    s += "\tans[0][4]=" + acosS(_d1[2][2]) + ";\n\n";
    s += "\tans[1][4]=" + acosS(_d2[2][2]) + ";\n\n";

    s += "\treturn ans;\n}";
    if (isValidEquationsPrctice(s))
        std::cout << s << '\n';
}