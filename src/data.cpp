#include "hummanArm/data.hpp"

void printMat(const Mat &m)
{

    for (const auto &x : m)
    {
        for (const auto &y : x)
        {
            std::cout << y << " ";
        }
        std::cout << '\n';
    }
}
void printPos(const Position &pos)
{
    std::cout << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    std::cout << "Orientation: (Roll: " << pos.roll << ", Pitch: " << pos.pitch << ", Yaw: " << pos.yaw << ")\n";
}