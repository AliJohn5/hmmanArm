import numpy as np
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
from robot import robot_chain

ang = [0,0,0,0,np.pi,0]
fk = robot_chain.forward_kinematics(ang)

print('####')
print(fk)
print('####')


target_position = [-0.5, -0.2, -0.2]
target_or = [-1,0,0]
ik_solution = robot_chain.inverse_kinematics(target_position)
print(ik_solution)
fk_solution = robot_chain.forward_kinematics(ik_solution)



print("Final transformation matrix from forward kinematics:\n", fk_solution)

def plot_robot(chain, joint_angles):
    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)  
    fig.set_figwidth(13) 
    s = [0,0 ,0, 0 ,0 ,0] 
    robot_chain.plot(ik_solution, ax, target=target_position)
    plt.xlim(-0.7, 0.7)
    plt.ylim(-0.7, 0.7)
    ax.set_zlim(-0.9, 0.9)
    plt.show()



plot_robot(robot_chain, ik_solution)
