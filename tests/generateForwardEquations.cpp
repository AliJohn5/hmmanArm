#include <hummanArm/sympole.hpp>

int main()
{
    MatS forward = generateForwardEquations(true , 5);

    for (size_t i = 0; i < 4; ++i)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            if (!isValidEquationsPrctice(forward[i][j]))
            {
                std::cerr << "ERROR in equation " << i << ", " << j << " : \n\n " << forward[i][j] << "\n\n";
                exit(-1);
            }
        }
    }

    std::string myf = generateForwardEquationsFunction(3);
    std::cout << myf << '\n';

    return 0;
}