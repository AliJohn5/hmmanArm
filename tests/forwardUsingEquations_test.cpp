#include <hummanArm/math.hpp>

int main()
{
    int n = 100000;
    Robot r;

    for (size_t _ = 0; _ < n; _++)
    {
        Ang ang = {
            ((rand() % 31419) + 0.0) / 10000 - 3.14 / 2,
            ((rand() % 31419) + 0.0) / 10000 - 3.14 / 2,
            ((rand() % 31419) + 0.0) / 10000 - 3.14 / 2,
            ((rand() % 31419) + 0.0) / 10000 - 3.14 / 2,
            ((rand() % 31419) + 0.0) / 10000 - 3.14 / 2,
        };

        Mat realForward = r.fk(ang);
        Mat newForward = forwardUsingEquations(ang);

        for (size_t i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < 4; j++)
            {
                if (abs(realForward[i][j] - newForward[i][j]) > 0.000001)
                {
                    std::cout << "ERROR in test " << _ << " :\nangles are:\n";
                    for (auto &x : ang)
                    {
                        std::cout << x << " ";
                    }
                    std::cout << '\n';
                    std::cout << "real ans is :\n";
                    printMat(realForward);

                    std::cout << "you get :\n";
                    printMat(newForward);

                    exit(-1);
                }
            }
        }
    }

    std::cout << "OK\n";
    return 0;
}