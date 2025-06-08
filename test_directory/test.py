import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from scipy.optimize import least_squares
from Trajectory_Planner import *

# 1) 관절 한계 (라디안)
MIN_LIMITS = np.array([0.0,
    np.deg2rad(-100), np.deg2rad(-105),
    np.deg2rad(-135), np.deg2rad(-115),
])
MAX_LIMITS = np.array([0.0,
    np.deg2rad( 100), np.deg2rad(  45),
    np.deg2rad(  30), np.deg2rad(   0),
])

# 2) 로봇 체인 정의
CHAIN = Chain(name='4DOF_arm', links=[
    OriginLink(),
    DHLink(d=120.75, a=0.0,   alpha=np.pi/2, theta=0),
    DHLink(d=0.0,   a=125.25, alpha=0.0,     theta=np.pi/2),
    DHLink(d=0.0,   a=110.25, alpha=0.0,     theta=0),
    DHLink(d=0.0,   a=98.5,  alpha=0.0,     theta=0),
    DHLink(d=0.0,   a=45.0,  alpha=-np.pi/2,     theta=0)
], active_links_mask=[False, True, True, True, True])

# 3) 오차 함수 (orientation 제약 포함)
def _error(q_active, target):
    full_q = np.hstack(([0.0], q_active))
    T = CHAIN.forward_kinematics(full_q, full_kinematics=True)[-1]
    pos_err = T[:3, 3] - target
    ori_err = T[:3, 0] - np.array([0.0, 0.0, -1.0])
    return np.hstack((pos_err, ori_err * 0.5))

# 4) Inverse Kinematics 함수 (외부에서 사용할 수 있게 그대로 유지)
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

start = np.array([100, 100, 150])
end   = np.array([100, -100, 100])

# ---- Trajectory 생성 및 IK 실행 ----
initial_q_full = (0, 0, np.deg2rad(40), np.deg2rad(-80), np.deg2rad(-50))
path = np.linspace(start, end, 100)  # 원하는 Trajectory 사용
raw_trajectory = inverse_kinematics(path, initial_q_full=initial_q_full)  # 여기서 결과 획득

# ---- Animation 시각화 함수 ----
def get_positions(q4):
    mats = CHAIN.forward_kinematics([0, *q4], full_kinematics=True)
    return np.array([m[:3, 3] for m in mats])

def get_frame(q4):
    T = CHAIN.forward_kinematics([0, *q4], full_kinematics=True)[-1]
    return T[:3, 3], T[:3, 0], T[:3, 2]

# ---- 애니메이션 ----
fig = plt.figure()
ax  = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_xlim(-300,300); ax.set_ylim(-300,300); ax.set_zlim(0,400)
ax.view_init(elev=30, azim=45)

pos0 = get_positions(raw_trajectory[0])
line, = ax.plot(pos0[:,0], pos0[:,1], pos0[:,2], '-o', lw=2)
pe, xh, zh = get_frame(raw_trajectory[0]); scale=50
qx = ax.quiver(*pe, *xh, length=scale, normalize=True, color='yellow')
qz = ax.quiver(*pe, *zh, length=scale, normalize=True, color='blue')

def update(i):
    q4 = raw_trajectory[i]
    pos = get_positions(q4)
    line.set_data(pos[:,0], pos[:,1])
    line.set_3d_properties(pos[:,2])
    global qx, qz
    qx.remove(); qz.remove()
    pe, xh, zh = get_frame(q4)
    qx = ax.quiver(*pe, *xh, length=scale, normalize=True, color='yellow')
    qz = ax.quiver(*pe, *zh, length=scale, normalize=True, color='blue')
    return line, qx, qz

anim = FuncAnimation(fig, update, frames=raw_trajectory.shape[0], interval=100, blit=False)
plt.show()
