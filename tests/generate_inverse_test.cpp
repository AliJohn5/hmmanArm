#include <hummanArm/math.hpp>
#include <random>
#include <fstream>
#include <istream>

int main()
{

    Robot r;
    srand(time(0));
    freopen("../outin/inverse_test.txt", "w", stdout);
    int n = 1000000;
    std::cout << n << '\n';

    for (size_t i = 0; i < n; i++)
    {

        Ang ang;
        for (auto &a : ang)
            a = ((rand() % (int)(2 * M_PI * 10000)) - M_PI * 10000) / 10000;

        Mat a = r.fk(ang);

        for (size_t i1 = 0; i1 < 5; i1++)
        {
            for (size_t i2 = 0; i2 < 5; i2++)
            {
                std::cout << a[i1][i2] << " ";
            }
        }
    }

    return 0;
}