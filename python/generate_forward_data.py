import numpy as np
import random
from ikpy.chain import Chain
import numpy as np
from robot import robot_chain

def get_position_and_rotation(fk_solution):
    x = fk_solution[0, 3]
    y = fk_solution[1, 3]
    z = fk_solution[2, 3]

    rotation_matrix = fk_solution[:3, :3]

    pitch = np.arcsin(-rotation_matrix[2, 0])
    roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    return {
        'x': x, 'y': y, 'z': z,
        'roll': np.degrees(roll),
        'pitch': np.degrees(pitch),
        'yaw': np.degrees(yaw)
    }

def generate_random_angles():
    angles = [random.uniform(-np.pi, np.pi) for _ in range(5)]
    angles.append(0.0)
    return angles


file = open("../outin/forward.txt","w")
n = 10000
print(n,file=file)

for i in range(n):
    ang = generate_random_angles()
    fk_solution = robot_chain.forward_kinematics(ang)
    data = get_position_and_rotation(fk_solution)
    print(ang[0],ang[1],ang[2],ang[3],ang[4]
        ,data['x'],data['y'],data['y'],
        data['roll'],data['pitch'],data['yaw'],
        file=file)
    
file.close()