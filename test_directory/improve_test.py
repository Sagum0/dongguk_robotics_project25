# example_run_with_z_constraint.py

import numpy as np
from ikpy.chain import Chain
from ikpy.link  import OriginLink, DHLink
from Trajectory_Planner import TrajectoryPlanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# # 관절 각도 제한 (라디안)
# min_limits = np.array([
#     0.0,
#     np.deg2rad(-100),  # 1번 관절
#     np.deg2rad(-125),  # 2번 관절
#     np.deg2rad(-125),  # 3번 관절
#     np.deg2rad(-125),  # 4번 관절
# ])
# max_limits = np.array([
#     0.0,
#     np.deg2rad( 100),
#     np.deg2rad( 45),
#     np.deg2rad( 30),
#     np.deg2rad( 30),
# ])

def generate_circle_with_transition(start, center, radius, z, 
                                    num_circle=100, num_trans=20):
    """
    start: (xs, ys, zs) 현재 로봇의 시작 좌표
    center: (x0, y0) 원의 중심
    radius: 원의 반지름
    z: 원 궤적 높이 (xy평면의 z)
    num_circle: 원 위 포인트 개수
    num_trans: 이동 구간(linear) 포인트 개수
    """
    xs, ys, zs = start
    x0, y0 = center

    # 1) 시작점에서 원 위로 이동: 
    #    원 위 근접점(angle0), 그리고 보간
    angle0 = np.arctan2(ys - y0, xs - x0)
    x_on = x0 + radius * np.cos(angle0)
    y_on = y0 + radius * np.sin(angle0)
    z_on = z

    # 선형 보간
    trans_t = np.linspace(0, 1, num_trans)
    x_trans = xs + (x_on - xs) * trans_t
    y_trans = ys + (y_on - ys) * trans_t
    z_trans = zs + (z_on - zs) * trans_t
    trans_path = np.vstack((x_trans, y_trans, z_trans)).T

    # 2) 원 궤적 생성 (angle0에서 시작)
    theta = np.linspace(angle0, angle0 + 2*np.pi, num_circle, endpoint=False)
    x_circle = x0 + radius * np.cos(theta)
    y_circle = y0 + radius * np.sin(theta)
    z_circle = np.full_like(theta, z)
    circle_path = np.vstack((x_circle, y_circle, z_circle)).T

    # 두 구간 연결
    full_path = np.vstack((trans_path, circle_path))
    return full_path

# 1) IK 체인 정의 (2번 관절에 π/2 오프셋 적용)
arm_chain = Chain(name='4DOF_arm', links=[
    OriginLink(),  # index 0: 베이스 (fixed)
    DHLink(d=0.1075, a=0.0,    alpha=np.pi/2, theta=0),
    DHLink(d=0.0,    a=0.1135, alpha=0.0,     theta=np.pi/2),  # θ₂ offset
    DHLink(d=0.0,    a=0.1085, alpha=0.0,     theta=0),
    DHLink(d=0.0,    a=0.0845, alpha=0.0,     theta=0),
])

# 2) 베이스 링크 제외
arm_chain.active_links_mask[0] = False

# # 3) 궤적 생성
# start = [0.15, 0.2, 0.10]
# end   = [0.15,  -0.15, 0.10]
# planner     = TrajectoryPlanner(start_point=start, end_point=end, num_points=100)
# path_points = planner.plan()  # (50, 3)
start   = (-0.2, 0.1, 0.1)   # 현재 로봇팔 위치
center = (0.15, 0.15)     # 예: x=0.3, y=0.2
radius = 0.07           # 반지름 0.1m
z = 0.0               # 높이 0.15m
path = generate_circle_with_transition(
        start, center, radius, z,
        num_circle=30, num_trans=10
    )

# 4) 초기 추정값: 체인 전체 링크 수만큼 0으로 세팅
prev_q = np.zeros(len(arm_chain.links))  # 길이 5

q_list = []

# 5) 첫 지점: 순수 위치 IK (orientation_mode 없이)
first_point = path[0]
q_full = arm_chain.inverse_kinematics(
    target_position  = first_point,   # 위치만
    initial_position = prev_q         # zero seed 사용
)
q_list.append(q_full[1:])             # 베이스 제외
prev_q = q_full                        # 다음 단계 초기값

# 6) 나머지 지점들: Z축 제약 IK
for p in path[1:]:
    q_full = arm_chain.inverse_kinematics(
        target_position   = p,
        # orientation_mode  = "Z",      # End Effector z축을 전역 z축과 정렬
        initial_position  = prev_q
    )
    # q_full = np.clip(q_full, min_limits, max_limits)
    q_list.append(q_full[1:])       # [q1, q2, q3, q4]
    prev_q = q_full                  # 연속성 보장

# 7) 결과 매트릭스
q_matrix = np.array(q_list)          # shape: (50, 4)
print("q_matrix shape:", q_matrix.shape)
# print(q_matrix)

dt = 0.001
T_seg = 0.1
steps = int(T_seg / dt) + 1
t = np.linspace(0, T_seg, steps)

s = 10*(t/T_seg)**3 - 15*(t/T_seg)**4 + 6*(t/T_seg)**5

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
print(profiles.shape)

link_DH = np.array([
    [0.1075, 0.0000,  np.pi/2],
    [0.0000, 0.0985,  0.0     ],
    [0.0000, 0.0985,  0.0     ],
    [0.0000, 0.0845,  0.0     ]
])

# ① 관절별 고정 오프셋 (θ₂에만 π/2)
theta_offset = np.array([0.0, np.pi/2, 0.0, 0.0])

def dh(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1]
    ])

def fk(q):
    T = np.eye(4)
    points = [T[:3,3].copy()]
    for i in range(4):
        d, a, alpha = link_DH[i]
        theta = q[i] + theta_offset[i]
        T = T @ dh(a, alpha, d, theta)
        points.append(T[:3,3].copy())
    return T, np.stack(points)


fig = plt.figure(); ax = fig.add_subplot(111, projection='3d')
ax.set_title(" "); ax.set_xlim(-0.3,0.3); ax.set_ylim(-0.3,0.3); ax.set_zlim(0,0.4)
line, = ax.plot([],[],[], 'o-', lw=2)
def update(k):
    _, pts = fk(profiles[k])
    line.set_data(pts[:,0], pts[:,1])
    line.set_3d_properties(pts[:,2])
    return line,
ani = FuncAnimation(fig, update, frames=profiles.shape[0], interval = dt * 1000, blit=True, repeat = False)
plt.show()