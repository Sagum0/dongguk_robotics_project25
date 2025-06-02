import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from scipy.optimize import least_squares

# 1) 관절 한계 (라디안)
MIN_LIMITS = np.array([0.0,
    np.deg2rad(-100), np.deg2rad(-120),
    np.deg2rad(-120), np.deg2rad(-110),
])
MAX_LIMITS = np.array([0.0,
    np.deg2rad( 100), np.deg2rad(  45),
    np.deg2rad(  30), np.deg2rad(   0),
])

# 2) 로봇 체인 정의
CHAIN = Chain(name='4DOF_arm', links=[
    OriginLink(),
    DHLink(d=107.5, a=0.0,   alpha=np.pi/2, theta=0),
    DHLink(d=0.0,   a=113.5, alpha=0.0,     theta=np.pi/2),
    DHLink(d=0.0,   a=108.5, alpha=0.0,     theta=0),
    DHLink(d=0.0,   a=84.5,  alpha=0.0,     theta=0),
], active_links_mask=[False, True, True, True, True])

# 3) 오차 함수: x_H 축이 항상 -Z 방향을 보도록 orientation 제약 추가
def _error(q_active, target):
    full_q = np.hstack(([0.0], q_active))
    T = CHAIN.forward_kinematics(full_q, full_kinematics=True)[-1]
    pos_err = T[:3, 3] - target
    ori_err = T[:3, 0] - np.array([0.0, 0.0, -1.0])
    return np.hstack((pos_err, ori_err * 0.5))

# 4) raw 궤적 IK 함수 (스플라인 없이 순수 least_squares 결과)
def inverse_kinematics(path, initial_q_full=None):
    path = np.asarray(path)
    N = path.shape[0]
    n_joints = len(CHAIN.links)

    # 초기값 설정
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

    # 각 경유점에 대해 least_squares 최적화
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

    # 활성 관절(1~4번) 각도만 반환
    raw_trajectory = all_q[:, 1:]
    return raw_trajectory