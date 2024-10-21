#include <hummanArm/math.hpp>

int main()
{
#ifdef _WIN32

    system("py ../python/generate_forward_data.py");

#else

    system("python3 ../python/generate_forward_data.py");
#endif

    freopen("../outin/forward.txt", "r", stdin);
    int n;
    std::cin >> n;
    double dif = 0;

    for (size_t i = 0; i < n; i++)
    {
        Ang v;
        for (auto &x : v)
            std::cin >> x;
        Position temp;
        std::cin >> temp.x >> temp.y >> temp.z >> temp.roll >> temp.pitch >> temp.yaw;
        Position ans = getPosition(forward(v));

        /*for (size_t i = 0; i < 5; i++)
        {
            std::cout << v[0] << " ";
        }*/
        // std::cout<<'\n';

        // printPos(ans);
        // printPos(temp);
        // return 0;

        dif = std::max(dif, (double)abs(ans.x - temp.x));
        dif = std::max(dif, (double)abs(ans.y - temp.y));
        dif = std::max(dif, (double)abs(ans.z - temp.z));
        dif = std::max(dif, (double)abs(ans.pitch - temp.pitch));
        dif = std::max(dif, (double)abs(ans.yaw - temp.yaw));
        dif = std::max(dif, (double)abs(ans.roll - temp.roll));

        if (dif > 1)
        {
            std::cout << i << '\n';
            for (auto &x : v)
                std::cout
                    << x << " ";
            std::cout << '\n';

            exit(-1);
        }
    }

    std::cout << dif << std::endl;
}