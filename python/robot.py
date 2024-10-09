from ikpy.chain import Chain
import numpy as np
from ikpy.link import  URDFLink

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