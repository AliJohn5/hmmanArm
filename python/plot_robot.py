import numpy as np
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
from robot import robot_chain
from robot import get_position_and_rotation



fk_solution = [0,0,710]
ori = [0,0,0]
s = [2.9361 ,-1.10791, 2.55291, -3.11372, 2.71204, 0]

fk = robot_chain.forward_kinematics(s)
target_or = [
    [0.751702 ,-0.334921 ,-0.56813],
    [-0.319699, -0.938521, 0.130273 ],
    [-0.576833, 0.083704, -0.812562 ],
    ]
target = [-395.192,83.2015,43.1121]


ik_solution = robot_chain.inverse_kinematics(target,target_orientation=target_or,orientation_mode='all')

fk = robot_chain.forward_kinematics(ik_solution)


print(fk)

print(ik_solution)

def plot_robot(chain, joint_angles):
    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)  
    fig.set_figwidth(13) 
    robot_chain.plot(ik_solution, ax, target=target,)
    plt.xlim(-800, 800)
    plt.ylim(-800, 800)
    ax.set_zlim(-900, 900)
    plt.show()



plot_robot(robot_chain, s)

