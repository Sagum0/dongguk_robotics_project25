# example_run_with_z_constraint.py

import numpy as np
from ikpy.chain import Chain
from ikpy.link  import OriginLink, DHLink
from trajectory_planner import TrajectoryPlanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 1) IK 체인 정의 (2번 관절에 π/2 오프셋 적용)
arm_chain = Chain(name='4DOF_arm', links=[
    OriginLink(),  # index 0: 베이스 (fixed)
    DHLink(d=0.1075, a=0.0,    alpha=np.pi/2, theta=0),
    DHLink(d=0.0,    a=0.0985, alpha=0.0,     theta=np.pi/2),  # θ₂ offset
    DHLink(d=0.0,    a=0.0985, alpha=0.0,     theta=0),
    DHLink(d=0.0,    a=0.0845, alpha=0.0,     theta=0),
])

# 2) 베이스 링크 제외
arm_chain.active_links_mask[0] = False

# 3) 궤적 생성
start = [0.15, 0.2, 0.10]
end   = [0.15,  -0.15, 0.10]
planner     = TrajectoryPlanner(start_point=start, end_point=end, num_points=50)
path_points = planner.plan()  # (50, 3)

# 4) 초기 추정값: 체인 전체 링크 수만큼 0으로 세팅
prev_q = np.zeros(len(arm_chain.links))  # 길이 5

q_list = []

# 5) 첫 지점: 순수 위치 IK (orientation_mode 없이)
first_point = path_points[0]
q_full = arm_chain.inverse_kinematics(
    target_position  = first_point,   # 위치만
    initial_position = prev_q         # zero seed 사용
)
q_list.append(q_full[1:])             # 베이스 제외
prev_q = q_full                        # 다음 단계 초기값

# 6) 나머지 지점들: Z축 제약 IK
for p in path_points[1:]:
    q_full = arm_chain.inverse_kinematics(
        target_position   = p,
        # orientation_mode  = "Z",      # End Effector z축을 전역 z축과 정렬
        initial_position  = prev_q
    )
    q_list.append(q_full[1:])       # [q1, q2, q3, q4]
    prev_q = q_full                  # 연속성 보장

# 7) 결과 매트릭스
q_matrix = np.array(q_list)          # shape: (50, 4)
print("q_matrix shape:", q_matrix.shape)
print(q_matrix)

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
    _, pts = fk(q_matrix[k])
    line.set_data(pts[:,0], pts[:,1])
    line.set_3d_properties(pts[:,2])
    return line,
ani = FuncAnimation(fig, update, frames=100, interval=50, blit=True)
plt.show()