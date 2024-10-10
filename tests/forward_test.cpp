#include <hummanArm/math.hpp>

int main()
{

    Vec ang_rad = {
        +M_PI_2,
        1,
        -1,
        -1,
        +1};

    Mat result = forward(ang_rad);
    // std::cout << "Final Transformation Matrix:\n";

    printMat(result);

    // std::cout << "Final Position for:" << i << " ,"<<j << " \n";

    Position pos = getPosition(result);

    printPos(pos);

return 0;
}