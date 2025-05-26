import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from scipy.optimize import least_squares
from scipy.interpolate import make_interp_spline
from trajectory_planner import TrajectoryPlanner

# 1) 관절 한계 (라디안)
min_limits = np.array([0.0,
    np.deg2rad(-100), np.deg2rad(-120),
    np.deg2rad(-120), np.deg2rad(-120),
])
max_limits = np.array([0.0,
    np.deg2rad( 100), np.deg2rad(  45),
    np.deg2rad(  30), np.deg2rad(   0),
])

# 2) 로봇 체인 정의
chain = Chain(name='4DOF_arm', links=[
    OriginLink(),
    DHLink(d=107.5, a=0.0,   alpha=np.pi/2, theta=0),
    DHLink(d=0.0,   a=98.5,  alpha=0.0,     theta=np.pi/2),
    DHLink(d=0.0,   a=98.5,  alpha=0.0,     theta=0),
    DHLink(d=0.0,   a=84.5,  alpha=0.0,     theta=0),
], active_links_mask=[False, True, True, True, True])

# 3) 오차 함수 (4개 변수 → full 5개 변환)
def ik_error(q_active, target):
    full_q = np.hstack(([0.0], q_active))
    T = chain.forward_kinematics(full_q, full_kinematics=True)[-1]
    pos_err = T[:3, 3] - target
    ori_err = T[:3, 0] - np.array([0, 0, -1.0])
    return np.hstack((pos_err, ori_err * 0.5))

# 4) 경로 생성
start = [20, 150, 50]
end   = [90, 150, 100]
N = 100
planner = TrajectoryPlanner(start_point=start, end_point=end, num_points=N)
path = np.array(planner.plan())

# 5) 첫 점은 IKPY의 기본 inverse_kinematics로 계산하여 초기값 확보
raw_full0 = chain.inverse_kinematics(
    target_position=path[0],
    initial_position=[0]*len(chain.links)
)
q_active0 = raw_full0[1:]  # 첫 점의 1~4번 관절
all_q = np.zeros((N, 5))
all_q[0, 0]   = 0.0
all_q[0, 1:] = q_active0

# 6) 나머지 점에 대해 least_squares 최적화
lower = min_limits[1:]
upper = max_limits[1:]
x0_active = q_active0.copy()

for i in range(1, N):
    res = least_squares(
        ik_error,
        x0=x0_active,
        bounds=(lower, upper),
        args=(path[i],),
        xtol=1e-6, ftol=1e-6
    )
    all_q[i, 0]   = 0.0
    all_q[i, 1:] = res.x
    x0_active     = res.x

# 7) 1~4번 관절만 추출 후 스플라인 보간
raw = all_q[:, 1:]  # shape (N,4)
t   = np.linspace(0, 1, N)
q_smooth = np.zeros_like(raw)
for j in range(4):
    spline = make_interp_spline(
        t, raw[:, j], k=5,
        bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)])
    )
    q_smooth[:, j] = spline(t)

# ★ 보간 후에도 반드시 첫·끝점 일치
q_smooth[0, :]  = raw[0, :]
q_smooth[-1, :] = raw[-1, :]

# 8) 시각화/애니메이션 헬퍼
def get_positions(q4):
    mats = chain.forward_kinematics([0, *q4], full_kinematics=True)
    return np.array([m[:3, 3] for m in mats])

def get_frame(q4):
    T = chain.forward_kinematics([0, *q4], full_kinematics=True)[-1]
    return T[:3, 3], T[:3, 0], T[:3, 2]

# 9) 애니메이션 그리기
fig = plt.figure()
ax  = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_xlim(-300,300); ax.set_ylim(-300,300); ax.set_zlim(0,400)
ax.view_init(elev=30, azim=45)

pos0 = get_positions(q_smooth[0])
line, = ax.plot(pos0[:,0], pos0[:,1], pos0[:,2], '-o', lw=2)
pe, xh, zh = get_frame(q_smooth[0]); scale=50
qx = ax.quiver(*pe, *xh, length=scale, normalize=True, color='yellow')
qz = ax.quiver(*pe, *zh, length=scale, normalize=True, color='blue')

def update(i):
    q4 = q_smooth[i]
    pos = get_positions(q4)
    line.set_data(pos[:,0], pos[:,1])
    line.set_3d_properties(pos[:,2])
    global qx, qz
    qx.remove(); qz.remove()
    pe, xh, zh = get_frame(q4)
    qx = ax.quiver(*pe, *xh, length=scale, normalize=True, color='yellow')
    qz = ax.quiver(*pe, *zh, length=scale, normalize=True, color='blue')
    return line, qx, qz

anim = FuncAnimation(fig, update, frames=N, interval=100, blit=False)
plt.show()
