#include <hummanArm/math.hpp>

int main()
{

    std::cout << "starting read" << std::endl;

    Robot r;

    std::cout << "starting ik.." << std::endl;

    Ang ik = r.ik(10,-5,160,{{1,1,1}},OrintationMode::x)[0];

    Mat fk = r.fk(ik);

    Position pos = getPosition(fk);

    for (auto &x : ik)
    {
        std::cout << x << " ";
    }
    std::cout << '\n';

    printMat(fk);
    printPos(pos);

    return 0;
}