import numpy as np
from ikpy.chain import Chain
from ikpy.link  import OriginLink, DHLink

# 관절 각도 제한 (라디안)
min_limits = np.array([
    0.0,
    np.deg2rad(-100),  # 1번 관절
    np.deg2rad(-125),  # 2번 관절
    np.deg2rad(-125),  # 3번 관절
    np.deg2rad(-125),  # 4번 관절
])
max_limits = np.array([
    0.0,
    np.deg2rad( 100),
    np.deg2rad( 45),
    np.deg2rad( 125),
    np.deg2rad( 125),
])

def inverse_k(path):
    # 로봇 체인 정의
    arm_chain = Chain(name='4DOF_arm', links=[
        OriginLink(),
        DHLink(d=107.5, a=0.0, alpha=np.pi/2, theta=0),
        DHLink(d=0.0,   a=98.5, alpha=0.0,   theta=np.pi/2),
        DHLink(d=0.0,   a=98.5, alpha=0.0,   theta=0),
        DHLink(d=0.0,   a=84.5, alpha=0.0,   theta=0),
    ])
    arm_chain.active_links_mask[0] = False

    path_points = path
    dt    = 0.001
    T_seg = 0.01
    steps = int(T_seg / dt) + 1
    t     = np.linspace(0, T_seg, steps)
    s     = 10*(t/T_seg)**3 - 15*(t/T_seg)**4 + 6*(t/T_seg)**5

    prev_q = np.zeros(len(arm_chain.links))
    q_list = []

    # 첫 번째 포인트 IK + 클램핑
    q_full = arm_chain.inverse_kinematics(
        target_position  = path_points[0],
        initial_position = prev_q
    )
    q_full = np.clip(q_full, min_limits, max_limits)
    q_list.append(q_full[1:])
    prev_q = q_full

    # 이후 모든 포인트 IK + 클램핑
    for p in path_points[1:]:
        q_full = arm_chain.inverse_kinematics(
            target_position   = p,
            initial_position  = prev_q
        )
        q_full = np.clip(q_full, min_limits, max_limits)
        q_list.append(q_full[1:])
        prev_q = q_full

    # joint-angle 행렬로 변환
    q_matrix = np.array(q_list)  # shape: (N, 4)
    print("q_matrix shape:", q_matrix.shape)

    # 시간 파라미터화 (세그먼트 단위)
    num_joint = q_matrix.shape[1]
    profiles  = []

    for j in range(num_joint):
        traj_j = []
        for k in range(len(q_matrix)-1):
            q0 = q_matrix[k,   j]
            q1 = q_matrix[k+1, j]
            dq = q1 - q0
            traj_j.append(q0 + dq * s)
        profiles.append(np.concatenate(traj_j))

    # 최종 (total_steps, num_joints)
    profiles = np.array(profiles).T
    print("profiles shape:", profiles.shape)

    return profiles
