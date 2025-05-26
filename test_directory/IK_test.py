import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from scipy.optimize import least_squares
from scipy.interpolate import make_interp_spline
from trajectory_planner import TrajectoryPlanner

# 1) 로봇 체인 정의
chain = Chain(name='4DOF_arm', links=[
    OriginLink(),
    DHLink(d=107.5, a=0.0,   alpha=np.pi/2),
    DHLink(d=0.0,   a=113.5, alpha=0.0, theta=np.pi/2),
    DHLink(d=0.0,   a=108.5, alpha=0.0),
    DHLink(d=0.0,   a=84.5,  alpha=0.0),
])

joint_min = np.array([
    np.deg2rad(-100),
    np.deg2rad(-125),
    np.deg2rad(-125),
    np.deg2rad(-125),
])
joint_max = np.array([
    np.deg2rad( 100),
    np.deg2rad(  45),
    np.deg2rad(  35),
    np.deg2rad(  35),
])

def ik_bounded(target_pos, q_prev_full):
    """
    target_pos: (3,)
    q_prev_full: (5,) Origin+4joint 이전 해
    returns: (5,) Origin+4joint bounded 해 (x_H 축은 전역 –Z 방향)
    """
    x0 = q_prev_full[1:]  # 초기 추정 (4,)

    def residual(q_vars):
        # q_full: [Origin=0] + 4개 관절
        q_full = [0.0] + list(q_vars)
        # full_kinematics=True 로 모든 링크 변환행렬 얻기
        mats = chain.forward_kinematics(q_full, full_kinematics=True)
        T_end = mats[-1]                      # 말단 변환
        # 1) 위치 오차
        pos_err = T_end[:3, 3] - target_pos
        # 2) x_H 축 방향(말단 회전행렬 첫 컬럼) 추출
        x_h_dir = T_end[:3, 0]
        # 원하는 방향: global –Z 축 = [0,0,-1]
        ori_err = x_h_dir - np.array([0.0, 0.0, -1.0])
        # 위치(3) + 자세(3) => 6차 residual
        return np.concatenate((pos_err, ori_err))

    sol = least_squares(
        residual,
        x0=x0,
        bounds=(joint_min, joint_max),
        xtol=1e-6, ftol=1e-6
    )
    return np.concatenate(([0.0], sol.x))


def compute_inverse_kinematics(path_points):
    q_matrix = []
    # origin(0)부터 tool(마지막)까지 포함한 6개 변수
    q_prev = np.zeros(len(chain.links))

    for target in path_points:
        # ➊ least_squares IK with bounds
        q_full = ik_bounded(target, q_prev)

        # ➋ 결과에서 Origin(0)과 tool 마지막 인덱스 제외하고, 4개만 저장
        q_matrix.append(q_full[1:])
        q_prev = q_full  # 다음 초기 추정으로 사용

    return np.array(q_matrix)  # shape (N,4)

def get_joint_positions(q):
    # forward_kinematics는 (joint_list, full_kinematics) 두 위치 인수를 받음
    mats = chain.forward_kinematics([0] + list(q), True)
    return np.array([m[:3, 3] for m in mats])

def get_end_effector_frame(q):
    mats = chain.forward_kinematics([0] + list(q), True)
    T = mats[-1]
    pos = T[:3, 3]
    R   = T[:3, :3]
    x_h = R @ np.array([1, 0, 0])
    z_h = R @ np.array([0, 0, 1])
    return pos, x_h, z_h


# 2) 경로 생성 및 IK 계산
start = [90, -150, 100]
end  = [90, 150, 100]
num_points = 100
plan  = TrajectoryPlanner(start_point=start, end_point=end, num_points=num_points)
path_points = plan.plan()
q_matrix   = compute_inverse_kinematics(path_points)

t = np.linspace(0.0, 1.0, num_points)
q_smooth = np.zeros_like(q_matrix)

for j in range(q_matrix.shape[1]):
    # 5차 스플라인, 시작·끝에서 1차(속도), 2차(가속도) 모두 0으로 고정
    spline = make_interp_spline(
        t, q_matrix[:, j], k=5,
        bc_type=(
            [(1, 0.0), (2, 0.0)],            # t=0 에서 (q', q'') = (0,0)
            [(1, 0.0), (2, 0.0)]             # t=1 에서 (q', q'') = (0,0)
        )
    )
    q_smooth[:, j] = spline(t) 

# 3) 3D 애니메이션 초기 셋업
fig = plt.figure()
ax  = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_xlim3d(-300, 300); ax.set_ylim3d(-300, 300); ax.set_zlim3d(0, 400)
ax.view_init(elev=30, azim=45)

# 링크 궤적(line)과 엔드이펙터 축(quiver) 초기 그리기
pos0 = get_joint_positions(q_smooth[0])
line, = ax.plot(pos0[:,0], pos0[:,1], pos0[:,2], '-o', lw=2)

pos_e, x_h0, z_h0 = get_end_effector_frame(q_smooth[0])
qh = 50  # 축 길이 스케일
quiver_x = ax.quiver(*pos_e, *x_h0, length=qh, normalize=True, color='yellow')
quiver_z = ax.quiver(*pos_e, *z_h0, length=qh, normalize=True, color='darkblue')

def update(frame):
    q = q_smooth[frame]
    # 1) 링크 궤적 업데이트
    pos = get_joint_positions(q)
    line.set_data(pos[:,0], pos[:,1])
    line.set_3d_properties(pos[:,2])
    # 2) 엔드이펙터 축 업데이트
    global quiver_x, quiver_z
    quiver_x.remove(); quiver_z.remove()
    pos_e, x_h, z_h = get_end_effector_frame(q)
    quiver_x = ax.quiver(*pos_e, *x_h, length=qh, normalize=True, color='yellow')
    quiver_z = ax.quiver(*pos_e, *z_h, length=qh, normalize=True, color='darkblue')
    return line, quiver_x, quiver_z

anim = FuncAnimation(fig, update, frames=len(q_smooth), interval=100, blit=False)
plt.show()
