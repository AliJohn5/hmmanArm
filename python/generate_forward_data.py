import numpy as np
import random
import numpy as np
from robot import robot_chain
from robot import get_position_and_rotation


def generate_random_angles():
    ang = []
    for i in range(5):
        ang.append(random.uniform(-np.pi, np.pi))
    
    ang.append(0)
    return ang


file = open("../outin/forward.txt","w")
n = 10000
print(n,file=file)

for i in range(n):
    ang = generate_random_angles()
    fk_solution = robot_chain.forward_kinematics(ang)
    data = get_position_and_rotation(fk_solution)
    print(ang[0],ang[1],ang[2],ang[3],ang[4]
        ,data['x'],data['y'],data['z'],
        data['roll'],data['pitch'],data['yaw'],
        file=file)
    
file.close()