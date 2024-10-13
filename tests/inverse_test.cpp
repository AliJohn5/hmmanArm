#include <hummanArm/math.hpp>

int main()
{

    std::cout << "starting read" << std::endl;

    Robot r;

    std::cout << "starting ik.." << std::endl;

    Mat mat = {
        {{
            -0.15325911118091118,
            0.08115180104779768,
            0.9848482268993207,
            10.645258998463213,
        } ,{
            0.551192230245765,
            0.83420434791661,
            0.01703617426898085,
            -5.139777749776439,
        } ,{
            -0.8201821566924855,
            0.545451639564611,
            -0.1725796591129475,
            159.66177579097828,
        }, {
            0.0,
            0.0,
            0.0,
            1.0,
        }}

    };

    Vec ik = r.ik(mat);
    

    Mat fk = r.fk(ik);

    Position pos = getPosition(fk);

    for (auto &x : ik)
    {
        std::cout << x << " ";
    }
    std::cout << '\n';

    printMat(fk);

    return 0;
}