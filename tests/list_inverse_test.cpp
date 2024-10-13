#include <hummanArm/math.hpp>
#include <random>

int main()
{
    std::cout << "starting read" << std::endl;
    Robot r;
    srand(time(0));

    float d = 0;
    int err = 0;

    for (size_t i = 0; i < 10000; i++)
    {
        /// std::cout << "test: " << i << '\n';
        // std::cout<<'\n';
        if (i % 100 == 0)
        {
            std::cout << "test: " << i << '\n';
        }

        Vec ang(5, 0);
        for (auto &a : ang)
            a = ((rand() % (int)(2 * M_PI * 10000)) - M_PI * 10000) / 10000;
        Mat a = r.fk(ang);
        Position target = getPosition(a);

        Vec new_ang = r.ik(a);
        Mat new_p = r.fk(new_ang);
        Position reah = getPosition(new_p);
        d = std::max(d, dist(target, reah));

        if (d > 5)
        {
            std::cerr << "wrong answer on test " << i << '\n';
            for (auto &a : ang)
                std::cout << a << " ";
            std::cout << '\n';
            std::cout << '\n';

            //  printMat(a);
            //  std::cout << '\n';

            //  for (auto &a : new_ang)
            //      std::cout << a << " ";
            //  std::cout << '\n';
            // std::cout << '\n';
            // printMat(new_p);
            // std::cout<<'\n';
            err++;
            exit(-1);
                }
        else
        {
            // std::cout << "OK: " << '\n';
        }
    }

    std::cout << "ERRORS are: " << err << '\n';
    std::cout << "max ERROR is: " << d << '\n';

    return 0;
}