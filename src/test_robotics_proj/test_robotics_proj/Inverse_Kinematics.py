# import numpy as np
# from ikpy.chain import Chain
# from ikpy.link import OriginLink, DHLink
# from scipy.optimize import least_squares
# from scipy.interpolate import make_interp_spline

# # 1) 로봇 체인 정의
# chain = Chain(name='4DOF_arm', links=[
#     OriginLink(),
#     DHLink(d=107.5, a=0.0,   alpha=np.pi/2),
#     DHLink(d=0.0,   a=98.5, alpha=0.0, theta=np.pi/2),
#     DHLink(d=0.0,   a=98.5, alpha=0.0),
#     DHLink(d=0.0,   a=84.5,  alpha=0.0),
# ])

# joint_min = np.array([
#     np.deg2rad(-100),
#     np.deg2rad(-125),
#     np.deg2rad(-125),
#     np.deg2rad(-115),
# ])
# joint_max = np.array([
#     np.deg2rad( 130),
#     np.deg2rad(  45),
#     np.deg2rad(  35),
#     np.deg2rad(  0),
# ])

# def ik_bounded(target_pos, q_prev_full):
#     """
#     target_pos: (3,)
#     q_prev_full: (5,) Origin+4joint 이전 해
#     returns: (5,) Origin+4joint bounded 해 (x_H 축은 전역 –Z 방향)
#     """
#     x0 = q_prev_full[1:]  # 초기 추정 (4,)

#     def residual(q_vars):
#         # q_full: [Origin=0] + 4개 관절
#         q_full = [0.0] + list(q_vars)
#         # full_kinematics=True 로 모든 링크 변환행렬 얻기
#         mats = chain.forward_kinematics(q_full, full_kinematics=True)
#         T_end = mats[-1]                      # 말단 변환
#         # 1) 위치 오차
#         pos_err = T_end[:3, 3] - target_pos
#         # 2) x_H 축 방향(말단 회전행렬 첫 컬럼) 추출
#         x_h_dir = T_end[:3, 0]
#         # 원하는 방향: global –Z 축 = [0,0,-1]
#         ori_err = x_h_dir - np.array([0.0, 0.0, -1.0])
#         # 위치(3) + 자세(3) => 6차 residual
#         return np.concatenate((pos_err, ori_err))

#     sol = least_squares(
#         residual,
#         x0=x0,
#         bounds=(joint_min, joint_max),
#         xtol=1e-6, ftol=1e-6
#     )
#     return np.concatenate(([0.0], sol.x))


# def compute_inverse_kinematics(path_points):
#     q_matrix = []
#     # origin(0)부터 tool(마지막)까지 포함한 6개 변수

#     zeros_full = np.zeros(len(chain.links))
#     first_full = ik_bounded(path_points[0], zeros_full)
#     q_matrix.append(first_full[1:])
#     q_prev = first_full

#     for target in path_points[1:]:
#         # ➊ least_squares IK with bounds
#         q_full = ik_bounded(target, q_prev)

#         # ➋ 결과에서 Origin(0)과 tool 마지막 인덱스 제외하고, 4개만 저장
#         q_matrix.append(q_full[1:])
#         q_prev = q_full  # 다음 초기 추정으로 사용

#     return np.array(q_matrix)  # shape (N,4)

# def inverse_kinematics(path, num_points):
#     q_matrix   = compute_inverse_kinematics(path)
#     print(q_matrix)
    
#     t = np.linspace(0.0, 5.0, num_points)
#     q_smooth = np.zeros_like(q_matrix)
    
#     for j in range(q_matrix.shape[1]):
#         # 5차 스플라인, 시작·끝에서 1차(속도), 2차(가속도) 모두 0으로 고정
#         spline = make_interp_spline(
#             t, q_matrix[:, j], k=5,
#             bc_type=(
#                 [(1, 0.0), (2, 0.0)],            # t=0 에서 (q', q'') = (0,0)
#                 [(1, 0.0), (2, 0.0)]             # t=1 에서 (q', q'') = (0,0)
#             )
#         )
#         q_smooth[:, j] = spline(t) 
#         # q_smooth.shape == (N,4)
#     for j in range(q_smooth.shape[1]):
#         q_smooth[:, j] = np.clip(q_smooth[:, j], joint_min[j], joint_max[j])

        
#     return q_smooth


# import numpy as np
# from ikpy.chain import Chain
# from ikpy.link  import OriginLink, DHLink

# # 관절 각도 제한 (라디안)
# min_limits = np.array([
#     0.0,
#     np.deg2rad(-100),  # 1번 관절
#     np.deg2rad(-120),  # 2번 관절
#     np.deg2rad(-120),  # 3번 관절
#     np.deg2rad(-120),  # 4번 관절
# ])
# max_limits = np.array([
#     0.0,
#     np.deg2rad( 100),
#     np.deg2rad( 45),
#     np.deg2rad( 30),
#     np.deg2rad( 0),
# ])

# def inverse_kinematics(path):
#     # 로봇 체인 정의
#     arm_chain = Chain(name='4DOF_arm', links=[
#         OriginLink(),
#         DHLink(d=107.5, a=0.0, alpha=np.pi/2, theta=0),
#         DHLink(d=0.0,   a=98.5, alpha=0.0,   theta=np.pi/2),
#         DHLink(d=0.0,   a=98.5, alpha=0.0,   theta=0),
#         DHLink(d=0.0,   a=84.5, alpha=0.0,   theta=0),
#     ])
#     arm_chain.active_links_mask[0] = False

#     path_points = path
#     dt    = 0.001
#     T_seg = 0.01
#     steps = int(T_seg / dt) + 1
#     t     = np.linspace(0, T_seg, steps)
#     s     = 10*(t/T_seg)**3 - 15*(t/T_seg)**4 + 6*(t/T_seg)**5

#     prev_q = np.zeros(len(arm_chain.links))
#     q_list = []

#     # 첫 번째 포인트 IK + 클램핑
#     q_full = arm_chain.inverse_kinematics(
#         target_position  = path_points[0],
#         initial_position = prev_q
#     )
#     q_full = np.clip(q_full, min_limits, max_limits)
#     q_list.append(q_full[1:])
#     prev_q = q_full

#     # 이후 모든 포인트 IK + 클램핑
#     for p in path_points[1:]:
#         q_full = arm_chain.inverse_kinematics(
#             target_position   = p,
#             initial_position  = prev_q
#         )
#         q_full = np.clip(q_full, min_limits, max_limits)
#         q_list.append(q_full[1:])
#         prev_q = q_full

#     # joint-angle 행렬로 변환
#     q_matrix = np.array(q_list)  # shape: (N, 4)
#     print("q_matrix shape:", q_matrix.shape)

#     # 시간 파라미터화 (세그먼트 단위)
#     num_joint = q_matrix.shape[1]
#     profiles  = []

#     for j in range(num_joint):
#         traj_j = []
#         for k in range(len(q_matrix)-1):
#             q0 = q_matrix[k,   j]
#             q1 = q_matrix[k+1, j]
#             dq = q1 - q0
#             traj_j.append(q0 + dq * s)
#         profiles.append(np.concatenate(traj_j))

#     # 최종 (total_steps, num_joints)
#     profiles = np.array(profiles).T
#     print("profiles shape:", profiles.shape)

#     print(profiles)
#     return profiles

# 파일: test_robotics_proj/Inverse_Kinematics.py
# 파일: test_robotics_proj/Inverse_Kinematics.py

import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from scipy.optimize import least_squares
from scipy.interpolate import make_interp_spline

# 1) 관절 한계 (라디안)
MIN_LIMITS = np.array([0.0,
    np.deg2rad(-100), np.deg2rad(-120),
    np.deg2rad(-120), np.deg2rad(-120),
])
MAX_LIMITS = np.array([0.0,
    np.deg2rad( 100), np.deg2rad(  45),
    np.deg2rad(  30), np.deg2rad(   0),
])

# 2) 로봇 체인 정의
CHAIN = Chain(name='4DOF_arm', links=[
    OriginLink(),
    DHLink(d=107.5, a=0.0,   alpha=np.pi/2, theta=0),
    DHLink(d=0.0,   a=113.5,  alpha=0.0,     theta=np.pi/2),
    DHLink(d=0.0,   a=108.5,  alpha=0.0,     theta=0),
    DHLink(d=0.0,   a=84.5,  alpha=0.0,     theta=0),
], active_links_mask=[False, True, True, True, True])

# 3) 오차 함수: q_active(4) → full_q(5) 변환
def _error(q_active, target):
    full_q = np.hstack(([0.0], q_active))
    T = CHAIN.forward_kinematics(full_q, full_kinematics=True)[-1]
    pos_err = T[:3, 3] - target
    ori_err = T[:3, 0] - np.array([0, 0, -1.0])
    return np.hstack((pos_err, ori_err * 0.5))

def inverse_kinematics(path, initial_q_full=None, total_time=5.0 ):
    """
    path: array_like of shape (N,3), 목표 (x,y,z) 좌표들
    initial_q_full: optional, array_like of shape (5,), 
                    [q0, q1, q2, q3, q4] 초기 관절각 (Origin 포함).
                    None일 경우 CHAIN.inverse_kinematics(path[0]) 결과 사용.
    returns: np.ndarray of shape (N,4), 1~4번 관절 각도(라디안) 궤적
    """
    path = np.array(path)
    N = len(path)

    # 4개 활성 관절에 대한 초기 추정
    if initial_q_full is None:
        raw0 = CHAIN.inverse_kinematics(
            target_position=path[0],
            initial_position=[0]*len(CHAIN.links)
        )
        q_active0 = raw0[1:]
        all_q_full = np.zeros((N, 5))
        all_q_full[0] = raw0
    else:
        initial_q_full = np.array(initial_q_full)
        q_active0 = initial_q_full[1:]
        all_q_full = np.zeros((N, 5))
        all_q_full[0] = initial_q_full

    lower = MIN_LIMITS[1:]
    upper = MAX_LIMITS[1:]
    x0_active = q_active0.copy()

    # 4~N 점에 대해 least_squares 최적화
    for i in range(1, N):
        res = least_squares(
            _error,
            x0=x0_active,
            bounds=(lower, upper),
            args=(path[i],),
            xtol=1e-6, ftol=1e-6
        )
        all_q_full[i, 0]   = 0.0
        all_q_full[i, 1:] = res.x
        x0_active         = res.x

    # 1~4번 관절만 추출 후 스플라인 보간
    raw = all_q_full[:, 1:]  # shape (N,4)
    t   = np.linspace(0, total_time, N)
    dt = t[1] - t[0]  # 시간 간격
    
    q_smooth = np.zeros_like(raw)
    for j in range(4):
        spline = make_interp_spline(
            t, raw[:, j], k=5,
            bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)])
        )
        q_smooth[:, j] = spline(t)

    # 보간 후에도 시작/끝점 원본과 일치하도록 강제
    q_smooth[0, :]  = raw[0, :]
    q_smooth[-1, :] = raw[-1, :]

    return q_smooth, dt