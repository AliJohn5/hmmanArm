
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
    // Vec links = {66.5, 335.8, 183, 55, 70};

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
        ans = ans * RYM(ang_rad[1]) * TZM(links[1]);
    }

    if (ang_rad.size() > 2)
    {
        ans = ans * RYM(ang_rad[2]) * TZM(links[2]);
    }

    if (ang_rad.size() > 3)
    {
        ans = ans * RZM(ang_rad[3]) * TZM(links[3]);
    }

    if (ang_rad.size() > 4)
    {
        ans = ans * RYM(ang_rad[4]) * TZM(links[4]);
    }

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

Mat getMat(const Position &pos)
{
    Mat ans = {
        {1, 0, 0, pos.x},
        {0, 1, 0, pos.y},
        {0, 0, 1, pos.z},
        {0, 0, 0, 1}

    };

    return ans * (RZM(pos.yaw) * RYM(pos.pitch) * RXM(pos.roll));
}

float dist(const Position &a, const Position &b)
{
    return (a.x - b.x) * (a.x - b.x) +
           (a.y - b.y) * (a.y - b.y) +
           (a.z - b.z) * (a.z - b.z);
}

float to_deg(float a)
{
    return a * 180 / M_PI;
}

void iterate(
    Position &target_pos,
    Position &starting_pos,

    Vec &starting_ang,
    float first_ind,
    float last_ind,
    float inc_ind)
{

    Vec ans = starting_ang;
    float d = dist(target_pos, starting_pos);

    for (float i1 = first_ind; i1 <= last_ind; i1 += inc_ind)
    {
        for (float i2 = first_ind; i2 <= last_ind; i2 += inc_ind)
        {
            for (float i3 = first_ind; i3 <= last_ind; i3 += inc_ind)
            {
                for (float i4 = first_ind; i4 <= last_ind; i4 += inc_ind)
                {
                    for (float i5 = first_ind; i5 <= last_ind; i5 += inc_ind)
                    {
                        Vec new_ang = starting_ang;
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

    Vec &starting_ang,
    float first_ind,
    float last_ind,
    float inc_ind)
{

    Vec ans = starting_ang;
    float d = dist(target_pos, starting_pos);

    // for (float i4 = first_ind; i4 <= last_ind; i4 += inc_ind)
    //{
    for (float i5 = first_ind; i5 <= last_ind; i5 += inc_ind)
    {
        Vec new_ang = starting_ang;
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

    Vec &starting_ang)
{

    Vec ans = starting_ang;
    float d = dist(target_pos, starting_pos);
    float inc = M_PI;
    int iter = 0;

    while (d > 1 && abs(inc) > 0.01)
    {
        iter++;
        Vec l_ang = ans;
        Vec r_ang = ans;

        l_ang[4] += inc;
        r_ang[4] -= inc;

        Position pl = getPosition(forward(l_ang));
        Position pr = getPosition(forward(r_ang));

        if (dist(pl, target_pos) < dist(pr, target_pos))
        {
            if (d > dist(pl, target_pos))
            {
                d = dist(pl, target_pos);
                ans = l_ang;
                inc *= 0.75;
            }
            else
                inc /= 2;
        }
        else
        {
            if (d > dist(pr, target_pos))
            {
                d = dist(pr, target_pos);
                ans = r_ang;
                inc *= 0.75;
            }
            else
                inc /= 2;
        }
    }
    // }

    //std::cout << "iter " << iter << '\n';
    starting_ang = ans;
}

void Best(
    Vec &old_ang,
    const Vec &new_ang,

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

Robot::Robot()
{
}

Mat Robot::fk(Vec ang)
{
    return forward(ang);
}

float Robot::getInc(
    Vec &current_ang,
    Position &current_pos,
    const Position &target_pos)

{
    Vec inc(current_ang.size(), 0);
    Vec temp(current_ang.size(), 0);
    Position p = current_pos;
    float d = dist(p, target_pos);

    for (int i = 0; i < inc.size(); ++i)
    {
        temp = current_ang;
        temp[i] += k;
        p = getPosition(forward(temp));
        float tt = dist(p, target_pos) - d;

        if (tt < -0.05)
        {
            inc[i] = k;
        }
        else if (tt > 0.05)
        {
            inc[i] = -k;
        }
    }

    for (int i = 0; i < inc.size(); ++i)
    {
        current_ang[i] += inc[i];
    }

    current_pos = getPosition(forward(current_ang));

    return dist(current_pos, target_pos);
}

float getdifRot(const Position &a, const Position &b)
{
    return (a.roll - b.roll) * (a.roll - b.roll) +
           (a.pitch - b.pitch) * (a.pitch - b.pitch) +
           (a.yaw - b.yaw) * (a.yaw - b.yaw);
}

Vec Robot::ik(Mat mat)
{

    Vec ans(5, 0);
    Mat mat4 = mat * TZM(-links[4]);

    ans[0] = atan2(mat4[1][3], mat4[0][3]);

    // sure

    // calculate r, and s
    float r = sqrt(mat4[0][3] * mat4[0][3] + mat4[1][3] * mat4[1][3]);
    float s = mat4[2][3] - links[0];

    // sure
    ans[2] = acos(
        (r * r + s * s - (links[2] + links[3]) * (links[2] + links[3]) - links[1] * links[1]) /
        (2 * (links[2] + links[3]) * links[1]));

    // sure
    ans[1] = M_PI_2 - (asin(
                           (
                               (links[2] + links[3]) * sin(M_PI - ans[2])) /
                           sqrt(s * s + r * r)) +
                       atan2(s, r));

    Mat mat2 = forward({ans[0], ans[1], ans[2]});

    std::swap(mat2[1][0], mat2[0][1]);
    std::swap(mat2[2][0], mat2[0][2]);
    std::swap(mat2[1][2], mat2[2][1]);

    Mat d = mat2 * mat4;

    ans[3] = atan2(d[1][2], d[0][2]);

    ans[4] = atan2(d[1][2], d[1][0]);

    // ans[4] = atan2(sqrt(1 - pow(sin(ans[0]) * d[0][2] - cos(ans[0]) * d[1][2], 2)), sin(ans[0]) * d[0][2] - cos(ans[0]) * d[1][2]);

    Position st = getPosition(forward(ans));
    Position targ = getPosition(mat);

    binary_search(targ, st, ans);

    /*iterates(
        targ, st, ans, -M_PI, M_PI, 50 * M_PI / 180);
    iterates(
        targ, st, ans, -25 * M_PI / 180, 50 * M_PI / 180, 10 * M_PI / 180);

    iterates(
        targ, st, ans, -5 * M_PI / 180, 5 * M_PI / 180, 2 * M_PI / 180);
*/
    return ans;
}
