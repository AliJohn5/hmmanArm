#include <hummanArm/math.hpp>

int main()
{

    Ang ang_rad = {
        M_PI_4,
        M_PI_4,
        M_PI_4,
        M_PI_4,
        M_PI_4};

    Mat result = forward(ang_rad);
    std::cout << "Final Transformation Matrix:\n";

    printMat(result);

    // std::cout << "Final Position for:" << i << " ,"<<j << " \n";

    Position pos = getPosition(result);

    printPos(pos);

return 0;
}