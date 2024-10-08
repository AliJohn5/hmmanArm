#include <hummanArm/math.hpp>

int main()
{
    Vec ang_rad = {
        M_PI / 4,
        M_PI / 6,
        M_PI / 2,
        M_PI / 4,
        M_PI / 2};

    Mat result = forward(ang_rad);
    std::cout << "Final Transformation Matrix:\n";

    printMat(result);

    std::cout << "Final Position:\n";

    Position pos = getPosition(result);

    printPos(pos);
    return 0;
}