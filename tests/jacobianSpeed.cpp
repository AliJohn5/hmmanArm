#include <hummanArm/math.hpp>
#include <Eigen/Dense>
#include <chrono>

int main()
{
    std::cout << "starting jk.." << std::endl;

    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    long long microsec = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
    int i = 0;
    Ang ang = {0, 0, 0, 0, 0};
    std::vector<double> joint_velocities, forward_velocities;

    while (microsec < 1000000)
    {
        joint_velocities = {0.05, 0.0, 0.0, 0.05, 0.0};
        forward_velocities = jacobianForward(joint_velocities, ang);

        ++i;
        finish = std::chrono::high_resolution_clock::now();
        microsec = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
    }

    std::cout << i << " Forward jacobian per sec\n";
    i = 0;
    std::vector<double> end_velocities, inverse_velocities;

    while (microsec < 2000000)
    {
        end_velocities = forward_velocities;
        inverse_velocities = jacobianInverse(end_velocities, ang);

        ++i;
        finish = std::chrono::high_resolution_clock::now();
        microsec = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
    }

    std::cout << i << " Inverse jacobian per sec\n";

    std::cout << "OK\n";
}