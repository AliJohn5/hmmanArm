import numpy as np
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
from robot import robot_chain
from robot import get_position_and_rotation



fk_solution = [40.506,-43.3206,-6.309]
ori = [0,0,0]
s = [-3.03339, 1.01789 ,-2.89209, -2.53319, -0.157093,0]

fk = robot_chain.forward_kinematics(s)

target =  [40.506,-43.3206,-6.309]
ik_solution = robot_chain.inverse_kinematics(target)

#fk = robot_chain.forward_kinematics(ik_solution)


print(fk)
print("{")
for i in fk:
    print("{")
    for j in i:
        print(j,end=" ,")
    print("}")

print("}")

print(fk)

def plot_robot(chain, joint_angles):
    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(9)  
    fig.set_figwidth(13) 
    robot_chain.plot(s, ax, target=target,)
    plt.xlim(-800, 800)
    plt.ylim(-800, 800)
    ax.set_zlim(-900, 900)
    plt.show()



plot_robot(robot_chain, s)

