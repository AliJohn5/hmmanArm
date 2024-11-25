#include <hummanArm/sympole.hpp>
#include <random>

int main()
{

    int n = 10000;

    for (size_t i = 0; i < n; i++)
    {
        std::string s(1000, '*');

        for (size_t j = 0; j < 50; j++)
        {
            int k = rand() % s.length();
            if (rand() % 2)
                s[k] = ' ';
            else
                s[k] = '\n';
        }

        strip(s);

        if (s.find_first_of(' ') != std::string::npos)
        {
            std::cerr << "ERROR in string " << i << " :" << s << '\n';

            exit(-1);
        }
        if (s.find_first_of('\n') != std::string::npos)
        {
            std::cerr << "ERROR in string " << i << " :" << s << '\n';
            exit(-1);
        }
    }

    std::cout << "ok\n";

    return 0;
}