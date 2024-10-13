from ikpy.chain import Chain
import numpy as np
from ikpy.link import  URDFLink

robot_chain = Chain(name='robot', links=[
            URDFLink(
                name = "base_link",
                origin_translation=[0, 0, 0.0],

                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],

            ),

            URDFLink(
                name="link_1",
                origin_translation=[0, 0, 66.5],
                origin_orientation=[0,0,0],
                rotation=[0, 1, 0], 

            ),
            URDFLink(
                name="link_2",
                origin_translation=[0, 0, 335.8], 
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0], 
                 
            ),

            URDFLink(
                name="link_3",
                origin_translation=[0, 0.0, 183], 
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],  
            ),
            URDFLink(
                name="link_4",
                origin_translation=[0.0, 0, 55], 
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],  
            ),


             
             URDFLink(
                name="link_5",
                origin_translation=[0, 0, 70], 
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 0],  
            ),
               


        ])


'''
 

             
'''



def get_position_and_rotation(fk_solution):
    x = fk_solution[0, 3]
    y = fk_solution[1, 3]
    z = fk_solution[2, 3]

    R1 = fk_solution[0, :3]
    R2 = fk_solution[1, :3]
    R3 = fk_solution[2, :3]

    pitch = np.arctan2(-R3[0], np.sqrt(R1[0]**2 + R2[0]**2))

    if np.cos(pitch) != 0:
        roll = np.arctan2(R2[2], R1[2])
        yaw = np.arctan2(R1[1], R1[0])
    else:
        roll = 0
        yaw = np.arctan2(-R2[0], R3[0])

    return {
        'x': x, 'y': y, 'z': z,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw
    }