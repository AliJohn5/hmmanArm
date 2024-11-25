#include <hummanArm/math.hpp>
#include <Eigen/Dense>

int main()
{
    Ang ang = {0, 0, 0, 0, 0};

    std::vector<double> joint_velocities = {0.05, 0.0, 0.0, 0.05, 0.0};
    std::vector<double> forward_velocities = jacobianForward(joint_velocities, ang);

    std::vector<double> end_velocities = forward_velocities;
    std::vector<double> inverse_velocities = jacobianInverse(end_velocities, ang);

    for (int i = 0; i < 5; ++i)
    {
        if (abs(joint_velocities[i] - inverse_velocities[i]) > 0.0001)
        {
            std::cout << "ERROR\n";
            exit(-1);
        }
    }

    std::cout << "OK\n";
}