import numpy as np
from ikpy.chain import Chain
from ikpy.link  import OriginLink, DHLink

def time_scaling_trap(num_points, T, accel_ratio=0.2):
    t  = np.linspace(0, T, num_points)
    Ta = accel_ratio * T
    Tc = T - 2*Ta
    s  = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < Ta:
            # 가속
            s[i] = 0.5*(ti/Ta)**2 * (Ta/T)
        elif ti < Ta + Tc:
            # 등속
            s[i] = (ti - 0.5*Ta) / T
        else:
            # 감속
            dt   = T - ti
            s[i] = 1 - 0.5*(dt/Ta)**2 * (Ta/T)

    return s

def inverse_k(path):
    arm_chain = Chain(name='4DOF_arm', links=[
        OriginLink(),
        DHLink(d=107.5,  a=0.0,   alpha=np.pi/2, theta=0),
        DHLink(d=0.0,    a=98.5,  alpha=0.0,     theta=np.pi/2),
        DHLink(d=0.0,    a=98.5,  alpha=0.0,     theta=0),
        DHLink(d=0.0,    a=84.5,  alpha=0.0,     theta=0),
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