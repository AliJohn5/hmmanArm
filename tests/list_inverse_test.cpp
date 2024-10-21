#include <hummanArm/math.hpp>
#include <random>

int main()
{
    std::cout << "starting read" << std::endl;
    freopen("../outin/inverse_test.txt", "r", stdin);
    int n = 100000;
    std::cin >> n;

    Robot r;

    double d = 0;
    int err = 0;

    for (size_t i = 0; i < n; i++)
    {
        if (i % 1000 == 0)
        {
            std::cout << "test: " << i << '\n';
        }

        Mat a;
        for (size_t i1 = 0; i1 < 5; i1++)
        {
            for (size_t i2 = 0; i2 < 5; i2++)
            {
                std::cin >> a[i1][i2];
            }
        }

        Position target = getPosition(a);

        Ang new_ang = r.ik(a)[0];
        Mat new_p = r.fk(new_ang);

        Position reah = getPosition(new_p);
        d = std::max(d, dist(target, reah));

        for (auto &aa : new_ang)
        {
            if (std::isnan(aa))
            {
                std::cerr << "nan detected on test " << i << '\n';
                for (auto &a : new_ang)
                    std::cout << a << " ";

                std::cout << '\n';
                printMat(a);

                exit(-1);
            }
        }

        if (d > 5)
        {
            std::cerr << "wrong answer on test " << i << '\n';
            std::cout << '\n';

            printMat(a);
            std::cout << '\n';

            for (auto &a : new_ang)
                std::cout << a << " ";
            std::cout << '\n';
            std::cout << '\n';
            printMat(new_p);
            std::cout << '\n';
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