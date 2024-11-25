#include <hummanArm/math.hpp>
#include <chrono>

int main()
{

    std::cout << "starting read" << std::endl;
    Robot r;
    std::cout << "starting ik.." << std::endl;

    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    long long microsec = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
    int i = 0;

    while (microsec < 1000000)
    {
        Ang ik = r.ik(10, -5, 160, {{1, 1, 1}}, OrintationMode::x)[0];
        ++i;

        finish = std::chrono::high_resolution_clock::now();
        microsec = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
    }

    // Mat fk = r.fk(ik);

    // Position pos = getPosition(fk);

    /*for (auto &x : ik)
    {
        std::cout << x << " ";
    }*/
    std::cout << i << " per second\n";

    // printMat(fk);
    // printPos(pos);

    return 0;
}