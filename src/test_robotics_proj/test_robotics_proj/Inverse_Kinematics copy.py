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