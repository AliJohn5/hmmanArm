import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils.plot import plot_chain
import ikpy.utils.plot as plot_utils

import numpy as np
import time
import math
from ikpy.link import OriginLink, URDFLink

robot_chain = Chain(name='robot', links=[
    URDFLink(
        name = "base_link",
        origin_translation=[0, 0, 0.0],

        origin_orientation=[0, 0, np.pi],
        rotation=[0, 0, 1],
    ),

    URDFLink(
        name="link_1",
        origin_translation=[0, 0, 0.0665],
        origin_orientation=[0, -np.pi/2, 0],
        rotation=[0, 0, 1], 
        
    ),
    
    URDFLink(
        name="link_2",
        origin_translation=[0.3358, 0, 0], 
        origin_orientation=[np.pi/2, 0, 0],
        rotation=[1, 0, 0],  
    ),

    URDFLink(
        name="link_3",
        origin_translation=[0.183, 0.0, 0], 
        origin_orientation=[0, -np.pi/2, 0],
        rotation=[1, 0, 0],  
    ),
    URDFLink(
        name="link_4",
        origin_translation=[0.0, 0, 0.055], 
        origin_orientation=[0, -np.pi/2, np.pi/2],
        rotation=[0, 0, 1],  
    ),
    URDFLink(
        name="link_5",
        origin_translation=[0.07, 0, 0], 
        origin_orientation=[0, 0, 0],
        rotation=[1, 0, 0],  
    ),


])

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
