import sympy as sp
import numpy as np
theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta1 theta2 theta3 theta4 theta5')  
links1,links2,links3,links4,links5 = sp.symbols('links1 links2 links3 links4 links5')  

# Trigonometric shorthand
sin = [sp.sin(theta1),sp.sin(theta2),sp.sin(theta3),sp.sin(theta4),sp.sin(theta5)]
cos = [sp.cos(theta1),sp.cos(theta2),sp.cos(theta3),sp.cos(theta4),sp.cos(theta5)]


# Forward kinematics: Transformation matrix elements
T = sp.Matrix.zeros(4, 4)  # Initialize the 4x4 transformation matrix
T[0, 0] = (((((cos[0] * cos[1]) * cos[2]) + ((cos[0] * sin[1]) * -sin[2])) * cos[3]) + (-sin[0] * sin[3])) * cos[4] + \
          ((((cos[0] * cos[1]) * sin[2]) + ((cos[0] * sin[1]) * cos[2])) * -sin[4])
T[0, 1] = (((((cos[0] * cos[1]) * cos[2]) + ((cos[0] * sin[1]) * -sin[2])) * -sin[3]) + (-sin[0] * cos[3]))
T[0, 2] = ((((cos[0] * cos[1]) * cos[2]) + ((cos[0] * sin[1]) * -sin[2])) * cos[3] + (-sin[0] * sin[3])) * sin[4] + \
          ((((cos[0] * cos[1]) * sin[2]) + ((cos[0] * sin[1]) * cos[2])) * cos[4])
T[0, 3] = (((((((cos[0] * cos[1]) * cos[2]) + ((cos[0] * sin[1]) * -sin[2])) * cos[3]) + (-sin[0] * sin[3])) * sin[4]) + \
          ((((cos[0] * cos[1]) * sin[2]) + ((cos[0] * sin[1]) * cos[2])) * cos[4])) * links5 + \
          (((((cos[0] * cos[1]) * sin[2]) + ((cos[0] * sin[1]) * cos[2])) * links4) + \
          ((((cos[0] * cos[1]) * sin[2]) + ((cos[0] * sin[1]) * cos[2])) * links3) + ((cos[0] * sin[1]) * links2))

T[1, 0] = (((((sin[0] * cos[1]) * cos[2]) + ((sin[0] * sin[1]) * -sin[2])) * cos[3]) + (cos[0] * sin[3])) * cos[4] + \
          ((((sin[0] * cos[1]) * sin[2]) + ((sin[0] * sin[1]) * cos[2])) * -sin[4])
T[1, 1] = (((((sin[0] * cos[1]) * cos[2]) + ((sin[0] * sin[1]) * -sin[2])) * -sin[3]) + (cos[0] * cos[3]))
T[1, 2] = ((((sin[0] * cos[1]) * cos[2]) + ((sin[0] * sin[1]) * -sin[2])) * cos[3] + (cos[0] * sin[3])) * sin[4] + \
          ((((sin[0] * cos[1]) * sin[2]) + ((sin[0] * sin[1]) * cos[2])) * cos[4])
T[1, 3] = (((((((sin[0] * cos[1]) * cos[2]) + ((sin[0] * sin[1]) * -sin[2])) * cos[3]) + (cos[0] * sin[3])) * sin[4]) + \
          ((((sin[0] * cos[1]) * sin[2]) + ((sin[0] * sin[1]) * cos[2])) * cos[4])) * links5 + \
          (((((sin[0] * cos[1]) * sin[2]) + ((sin[0] * sin[1]) * cos[2])) * links4) + \
          ((((sin[0] * cos[1]) * sin[2]) + ((sin[0] * sin[1]) * cos[2])) * links3) + ((sin[0] * sin[1]) * links2))

T[2, 0] = (((((-sin[1]) * cos[2]) + (cos[1] * -sin[2])) * cos[3]) * cos[4]) + \
          (((-sin[1] * sin[2]) + (cos[1] * cos[2])) * -sin[4])
T[2, 1] = (((-sin[1]) * cos[2]) + (cos[1] * -sin[2])) * -sin[3]
T[2, 2] = (((((-sin[1]) * cos[2]) + (cos[1] * -sin[2])) * cos[3]) * sin[4]) + \
          (((-sin[1] * sin[2]) + (cos[1] * cos[2])) * cos[4])
T[2, 3] = (((((((-sin[1]) * cos[2]) + (cos[1] * -sin[2])) * cos[3]) * sin[4]) + \
          (((-sin[1] * sin[2]) + (cos[1] * cos[2])) * cos[4])) * links5) + \
          ((((-sin[1] * sin[2]) + (cos[1] * cos[2])) * links4) + \
          ((((-sin[1] * sin[2]) + (cos[1] * cos[2])) * links3) + \
          ((cos[1] * links2) + links1)))

T[3, 3] = 1  # Homogeneous coordinate

# Extract position and Y-axis (orientation)
position = T[:3, 3]  # End-effector position
y_axis = T[:3, 1]  # Y-axis of the transformation matrix

# Jacobian Calculation
# Translational Jacobian: Partial derivatives of position w.r.t. each joint

theta = [theta1, theta2, theta3, theta4, theta5]
J_v = sp.Matrix([
    [sp.diff(position[0], theta[i]) for i in range(5)],  # ∂x/∂θi
    [sp.diff(position[1], theta[i]) for i in range(5)],  # ∂y/∂θi
    [sp.diff(position[2], theta[i]) for i in range(5)]   # ∂z/∂θi
])

# Rotational Jacobian: Y-axis contribution
J_omega = sp.Matrix([
    [sp.diff(y_axis[0], theta[i]) for i in range(5)],  # ∂y_x/∂θi
    [sp.diff(y_axis[1], theta[i]) for i in range(5)],  # ∂y_y/∂θi
    [sp.diff(y_axis[2], theta[i]) for i in range(5)]   # ∂y_z/∂θi
])

# Full Jacobian (6x5)
J = sp.Matrix.vstack(J_v, J_omega)

# Print the full Jacobian
#sp.pprint(J)

with open("../outin/jacobian.txt", "w") as f:
    for i in range(J.rows):
        for j in range(J.cols):
            f.write(f"j[{i}][{j}] = {J[i, j]};\n")



joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
mylinks= [66.5, 335.8, 183, 55, 70]

numerical_J = np.array(J.subs({
    theta1: joint_angles[0],
    theta2: joint_angles[1],
    theta3: joint_angles[2],
    theta4: joint_angles[3],
    theta5: joint_angles[4],

    links1: mylinks[0],
    links2: mylinks[1],
    links3: mylinks[2],
    links4: mylinks[3],
    links5: mylinks[4],
       
}).evalf(), dtype=float)





joint_velocities = np.array([0.05, 0.0, 0.0, 0.050, 0.0])

forward_velocities =   numerical_J @ joint_velocities
print(forward_velocities)
# [64.38, 0.0, .0, .0, .0, 0.]

desired_velocities = forward_velocities
recovered_velocities = np.linalg.pinv(numerical_J) @ desired_velocities
print("Recovered Velocities:", recovered_velocities)

assert np.allclose(recovered_velocities, joint_velocities,atol=1e-6), "Validation failed!"


'''
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main() {
    // Define a matrix (Jacobian)
    MatrixXd J(3, 2);
    J << 1, 2,
         3, 4,
         5, 6;

    // Calculate the pseudo-inverse of J
    MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();

    // Print the pseudo-inverse
    cout << "Pseudo-inverse of J:" << endl;
    cout << J_pinv << endl;

    return 0;
}

'''