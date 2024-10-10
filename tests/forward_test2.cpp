#include <hummanArm/math.hpp>

int main()
{
    freopen("../outin/forward.txt", "r", stdin);
    int n;
    std::cin >> n;
    float dif = 0;

    for (size_t i = 0; i < n; i++)
    {
        Vec v(5);
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

        dif = std::max(dif, (float)abs(ans.x  - temp.x));
        dif = std::max(dif, (float)abs(ans.y  - temp.y));
        dif = std::max(dif, (float)abs(ans.z  - temp.z));
        dif = std::max(dif, (float)abs(ans.pitch - temp.pitch));
        dif = std::max(dif, (float)abs(ans.yaw - temp.yaw));
        dif = std::max(dif, (float)abs(ans.roll - temp.roll));
    }

    std::cout << dif << std::endl;
}