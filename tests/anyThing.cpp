#include <hummanArm/math.hpp>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main()
{
    // Create and manipulate a matrix using Eigen
    MatrixXd A(3, 2);
    A << 1, 2,
        3, 4,
        5, 6;

    cout << "Matrix A:" << endl;
    cout << A << endl;

    return 0;
}
