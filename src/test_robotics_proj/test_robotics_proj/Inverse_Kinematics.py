import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from scipy.optimize import least_squares

# 1) 관절 한계 (라디안) - 5DOF
MIN_LIMITS = np.array([0.0,
    np.deg2rad(-100), np.deg2rad(-105),
    np.deg2rad(-135), np.deg2rad(-115), np.deg2rad(-20)
])
MAX_LIMITS = np.array([0.0,
    np.deg2rad( 100), np.deg2rad(  45),
    np.deg2rad(  30), np.deg2rad(   0), np.deg2rad(150)
])

# 2) 5자유도 로봇 체인 정의
CHAIN = Chain(name='5DOF_pen_arm', links=[
    OriginLink(),
    DHLink(d=120.75, a=0.0,   alpha=np.pi/2, theta=0),
    DHLink(d=0.0,   a=125.25, alpha=0.0,     theta=np.pi/2),
    DHLink(d=0.0,   a=130.25, alpha=0.0,     theta=0),
    DHLink(d=0.0,   a=98.5,   alpha=0.0,     theta=0),
    DHLink(d=0.0,   a=34.25,   alpha=np.pi/2, theta=0)
], active_links_mask=[False, True, True, True, True, True])

# 3) 오차 함수 (Z_H 방향 제약 포함)
def _error(q_active, target):
    full_q = np.hstack(([0.0], q_active))
    T = CHAIN.forward_kinematics(full_q, full_kinematics=True)[-1]
    pos_err = T[:3, 3] - target
    z_axis = T[:3, 2]   # 엔드이펙터의 z축 벡터
    ori_err = z_axis - np.array([0.0, 0.0, -1.0])
    return np.hstack((pos_err, ori_err * 0.5))

# 4) Inverse Kinematics 함수 (외부에서 그대로 사용)
def inverse_kinematics(path, initial_q_full=None):
    path = np.asarray(path)
    N = path.shape[0]
    n_joints = len(CHAIN.links)

    if initial_q_full is None:
        raw0 = CHAIN.inverse_kinematics(
            target_position=path[0],
            initial_position=[0.0] * n_joints
        )
        q_active0 = raw0[1:]
        all_q = np.zeros((N, n_joints))
        all_q[0] = raw0
    else:
        initial_q_full = np.asarray(initial_q_full)
        q_active0 = initial_q_full[1:]
        all_q = np.zeros((N, n_joints))
        all_q[0] = initial_q_full

    lower = MIN_LIMITS[1:]
    upper = MAX_LIMITS[1:]
    x0 = q_active0.copy()

    for i in range(1, N):
        res = least_squares(
            _error,
            x0,
            bounds=(lower, upper),
            args=(path[i],),
            xtol=1e-6, ftol=1e-6
        )
        all_q[i, 0] = 0.0
        all_q[i, 1:] = res.x
        x0 = res.x

    raw_trajectory = all_q[:, 1:]
    return raw_trajectory
