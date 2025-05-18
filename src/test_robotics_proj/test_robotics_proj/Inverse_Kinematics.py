import numpy as np
from ikpy.chain import Chain
from ikpy.link  import OriginLink, DHLink

def inverse_k(path):
    arm_chain = Chain(name='4DOF_arm', links=[
        OriginLink(),
        DHLink(d=107.5,  a=0.0,   alpha=np.pi/2, theta=0),
        DHLink(d=0.0,    a=98.5,  alpha=0.0,     theta=np.pi/2),
        DHLink(d=0.0,    a=98.5,  alpha=0.0,     theta=0),
        DHLink(d=0.0,    a=84.5,  alpha=0.0,     theta=0),
    ])
    
    path_points = path
    dt = 0.001
    T_seg = 0.01
    steps = int(T_seg / dt) + 1
    t = np.linspace(0, T_seg, steps)

    s = 10*(t/T_seg)**3 - 15*(t/T_seg)**4 + 6*(t/T_seg)**5
    
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
    #print(q_matrix)
    
    num_joint = q_matrix.shape[1]
    profiles = []
    
    for j in range(num_joint):
        traj_j = []
        for k in range(len(q_matrix)-1):
            q0 = q_matrix[k,   j]
            q1 = q_matrix[k+1, j]
            dq = q1 - q0
            
            traj_j.append(q0 + dq * s)
            
        profiles.append(np.concatenate(traj_j))

        # **반드시** numpy array로 변환하고 전치
    profiles = np.array(profiles).T  # shape: (total_steps, num_joints)
    
    print(profiles)
    return profiles