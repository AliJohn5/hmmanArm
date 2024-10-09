#include <hummanArm/math.hpp>

int main()
{
    Vec ang_rad = {
        0,
        0,
        0,
        0,
        0};

    Mat result = forward(ang_rad);
    std::cout << "Final Transformation Matrix:\n";

    printMat(result);

    std::cout << "Final Position:\n";

    Position pos = getPosition(result);

    printPos(pos);
    return 0;
}