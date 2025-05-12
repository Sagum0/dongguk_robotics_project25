import numpy as np
from ikpy.chain import Chain
from ikpy.link  import OriginLink, DHLink

def inverse_k(path):
    arm_chain = Chain(name='4DOF_arm', links=[
        OriginLink(),
        DHLink(d=0.1075, a=0.0,    alpha=np.pi/2, theta=0),
        DHLink(d=0.0,    a=0.0985, alpha=0.0,     theta=np.pi/2),
        DHLink(d=0.0,    a=0.0985, alpha=0.0,     theta=0),
        DHLink(d=0.0,    a=0.0845, alpha=0.0,     theta=0),
    ])
    
    path_points = path
    
    arm_chain.active_links_mask[0] = False

    prev_q = np.zeros(len(arm_chain.links))

    q_list = []

    first_point = path_points[0]
    q_full = arm_chain.inverse_kinematics(
        target_position  = first_point, 
        initial_position = prev_q 
    )
    q_list.append(q_full[1:])
    prev_q = q_full 

    for p in path_points[1:]:
        q_full = arm_chain.inverse_kinematics(
            target_position   = p,
            initial_position  = prev_q
        )
        q_list.append(q_full[1:])
        prev_q = q_full

    q_matrix = np.array(q_list)
    print("q_matrix shape:", q_matrix.shape)
    print(q_matrix)
    
    return q_matrix