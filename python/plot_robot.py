import numpy as np
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
from robot import robot_chain
from robot import get_position_and_rotation


ik_solution = [0,+np.pi/2,1,-1,-1,1]
fk_solution = robot_chain.forward_kinematics(ik_solution)
print(get_position_and_rotation(fk_solution))

#fk_solution = [335.8,0,66.5]
#ik_solution = robot_chain.inverse_kinematics(point)

def plot_robot(chain, joint_angles):
    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)  
    fig.set_figwidth(13) 
    s = [0,0 ,0, 0 ,0 ,0] 
    robot_chain.plot(ik_solution, ax, target=fk_solution,)
    plt.xlim(-800, 800)
    plt.ylim(-800, 800)
    ax.set_zlim(-900, 900)
    plt.show()



plot_robot(robot_chain, ik_solution)

