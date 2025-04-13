#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray

##############################
# 2. DH 파라미터 설정 (4DoF 로봇)
##############################
# 각 행은 [a, alpha, d, theta0]를 의미함.
D1 = 107.5
A2  = 98.5
A3  = 98.5
A4  = 84.5
ALPHA_1 = np.pi / 2

##############################
# 3. 기본 함수: DH 변환행렬 생성
##############################
def dh_transform(theta, d, a, alpha):
    """
    DH 파라미터를 받아 4x4 동차변환행렬을 생성합니다.
    입력:
      theta: 회전각 (rad)
      d:     평행이동 (mm)
      a:     링크 길이 (mm)
      alpha: 링크 꼬임각 (rad)
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

##############################
# 4. Forward Kinematics (정방향 기구학)
##############################
def forward_kinematics(joint_angles):
    """
    입력된 관절각(joint_angles, 길이 4 벡터)을 사용해
    끝 단(End Effector)의 최종 위치 변환행렬과 각 관절의 좌표 리스트를 계산합니다.
    반환:
      T04: 4x4 동차변환행렬 (최종 말단 좌표 포함)
      points: [base, p1, p2, p3, p4] 형태의 좌표 리스트 (각 관절 및 말단의 [x,y,z])
    """
    theta1, theta2, theta3, theta4 = joint_angles
    base_point = np.array([0, 0, 0])
    points = [base_point]
    
    # 0 -> 1
    T01 = dh_transform(theta1, D1, 0.0, ALPHA_1)
    p1 = T01[:3, 3]
    points.append(p1)
    
    # 1 -> 2
    T12 = dh_transform(theta2, 0.0, A2, 0.0)
    T02 = T01 @ T12
    p2 = T02[:3, 3]
    points.append(p2)
    
    # 2 -> 3
    T23 = dh_transform(theta3, 0.0, A3, 0.0)
    T03 = T02 @ T23
    p3 = T03[:3, 3]
    points.append(p3)
    
    # 3 -> 4 (끝 단)
    T34 = dh_transform(theta4, 0.0, A4, 0.0)
    T04 = T03 @ T34
    p4 = T04[:3, 3]
    points.append(p4)
    
    return T04, points

##############################
# 5. Jacobian 계산 (수치 미분 방식)
##############################
def jacobian_numerical(q, eps=1e-6):
    """
    관절 각도 벡터 q (길이 4)에 대해 수치 미분을 통해
    3x4 크기의 자코비안 행렬을 계산합니다.
    각 열은, 해당 관절각을 eps 만큼 변화시켰을 때
    말단의 [x,y,z] 좌표 변화량을 나타냅니다.
    """
    J = np.zeros((3, len(q)))
    T, _ = forward_kinematics(q)
    pos = T[:3, 3]
    for i in range(len(q)):
        dq = np.zeros_like(q)
        dq[i] = eps
        T_perturbed, _ = forward_kinematics(q + dq)
        pos_perturbed = T_perturbed[:3, 3]
        J[:, i] = (pos_perturbed - pos) / eps
    return J

##############################
# 6. DLS (Damped Least Squares) Jacobian 기반 Inverse Kinematics
##############################
def inverse_kinematics(target, q_init=np.zeros(4), lam=0.05, iters=100):
    """
    DLS Jacobian 역기구학 알고리즘을 사용해 목표 위치 target (3-vector, mm)
    에 도달하기 위한 관절각을 수치적으로 계산합니다.
    
    입력:
      target: 목표 위치 [x, y, z]
      q_init: 초기 관절각 (기본: [0,0,0,0])
      lam:    댐핑 계수 (특이점 안정화를 위해)
      iters:  최대 반복 횟수
    반환:
      관절각 q (4-vector)
    """
    q = q_init.copy()
    for _ in range(iters):
        T, _ = forward_kinematics(q)
        pos = T[:3, 3]
        e = target - pos  # 위치 오차 (mm)
        if np.linalg.norm(e) < 1e-3:
            return q
        J = jacobian_numerical(q)
        JT = J.T
        JJT = J @ JT  # 3x3 행렬
        damped_term = JJT + (lam**2) * np.eye(3)
        J_dls = JT @ np.linalg.inv(damped_term)
        dq = J_dls @ e
        q = q + dq
    if np.linalg.norm(e) >= 1e-3:
        raise ValueError("Inverse Kinematics did not converge!")
    return q

##############################
# 7. 보간 함수들: Cubic (5차 다항식) 및 Linear (선형) 보간
##############################
def cubic_interpolation(p0, pf, t, tf):
    """
    5차 다항식 보간법 (초기, 최종 속도 및 가속도 0)
      p(t) = p0 + (pf-p0)*(10*(t/tf)^3 - 15*(t/tf)^4 + 6*(t/tf)^5)
    """
    s = t / tf
    return p0 + (pf - p0) * (10 * s**3 - 15 * s**4 + 6 * s**5)

def linear_interpolation(p0, pf, t, tf):
    """
    선형 보간법:
      p(t) = p0 + (pf-p0)*(t/tf)
    """
    s = t / tf
    return p0 + (pf - p0) * s

def generate_trajectory(p0, pf, tf, num_frames, method='cubic'):
    """
    시작점 p0와 끝점 pf 사이를 보간법으로 연결하는 경로(trajectory)를 생성합니다.
    반환:
      (num_frames, 3) 모양의 numpy 배열 – 각 행은 [x,y,z]
    """
    traj = []
    ts = np.linspace(0, tf, num_frames)
    for t in ts:
        if method == 'cubic':
            p = cubic_interpolation(p0, pf, t, tf)
        elif method == 'linear':
            p = linear_interpolation(p0, pf, t, tf)
        else:
            raise ValueError("보간법 선택 오류: 'cubic' 또는 'linear'를 지정하세요.")
        traj.append(p)
    return np.array(traj)

##############################
# 8. 각 경로점에 대해 역기구학을 풀어 관절각 궤적 생성
##############################
def compute_joint_trajectory(traj):
    """
    생성된 3차원 경로 traj (각 행이 [x,y,z])에 대해,
    각 위치마다 DLS Jacobian IK를 풀어 관절각 벡터를 계산합니다.
    """
    joint_traj = []
    q_current = np.zeros(4)
    for point in traj:
        try:
            # 이전 관절각을 초기 추정치로 사용하여 수렴 가속
            q_current = inverse_kinematics(point, q_init=q_current)
        except ValueError as e:
            print(e)
            q_current = None
        joint_traj.append(q_current)
    return joint_traj

#########################################################################
# Control_Node 노드 구현
#########################################################################
class ControlNode(Node):
    def __init__(self):
        super().__init__('Control_Node')
        # (1) 모터 제어가 subscribe하는 토픽으로, 계산된 θ(관절각)를 발행합니다.
        self.theta_pub = self.create_publisher(Float32MultiArray, '/robotics_goal_position', 10)
        # (2) 모터 제어가 publish하는 현재 관절 상태를 구독합니다.
        self.create_subscription(Float32MultiArray, '/robotics_present_position', self.present_callback, 10)
        # (3) 외부에서 입력되는 목표 엔드 이펙터 위치(goal pose)를 구독합니다.
        #     (메시지 배열 길이가 3이면 [x,y,z]로 간주)
        self.create_subscription(Float32MultiArray, '/robotics_target_position', self.goal_callback, 10)
        
        # 타이머 주기 10ms
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # 현재 관절 상태 (초기값: [0,0,0,0])
        self.current_joint_angles = np.zeros(4)
        # 생성된 관절궤적 (θ 값들)와 진행 인덱스
        self.trajectory_joint = None
        self.traj_index = 0
        self.executing_trajectory = False
        
        # 허용 오차 (rad)
        self.eps = 0.05
        
        # 경로 생성을 위한 파라미터 (전체 이동 시간, 프레임 수)
        self.tf = 2.0  # 전체 이동 시간 (초)
        self.num_frames = int(self.tf / 0.01)  # 타이머 주기에 따른 프레임 수

    def present_callback(self, msg):
        """
        motor_control.py 노드가 publish 하는 현재 관절 상태를 업데이트합니다.
        메시지 data는 4개의 값으로 구성된 Float32MultiArray여야 합니다.
        """
        data = msg.data
        if len(data) >= 4:
            self.current_joint_angles = np.array(data[:4])
        else:
            self.get_logger().warn("present_position 메시지 길이 부족: {}".format(len(data)))

    def goal_callback(self, msg):
        """
        외부에서 목표 엔드 이펙터 위치(goal pose)가 입력되면, 현재 present_pose를 시작점으로 하여
        Cartesian 경로를 생성하고 각 경로점마다 역기구학으로 joint trajectory (θ 값들)를 계산합니다.
        여기서는 입력 메시지의 data 길이가 3이면 [x, y, z] 목표로 간주합니다.
        """
        data = msg.data
        if len(data) < 3:
            self.get_logger().warn("입력된 goal pose 메시지 길이가 3 미만입니다.")
            return
        
        goal_position = np.array(data[:3])
        # 현재 관절 상태로부터 엔드 이펙터의 현재 위치 계산
        T, _ = forward_kinematics(self.current_joint_angles)
        start_position = T[:3, 3]
        
        # 보간법 선택 (여기서는 예제로 cubic 사용)
        method = 'cubic'
        traj = generate_trajectory(start_position, goal_position, self.tf, self.num_frames, method)
        self.trajectory_joint = compute_joint_trajectory(traj)
        self.traj_index = 0
        self.executing_trajectory = True
        self.get_logger().info("새 목표 수신: {} 포인트 경로 생성 (start: {}, goal: {})".format(
            len(self.trajectory_joint), start_position, goal_position))
        
        # 첫 번째 θ 명령 즉시 발행 (유효한 값인 경우)
        if self.trajectory_joint and self.trajectory_joint[0] is not None:
            self.publish_theta(self.trajectory_joint[0])
        else:
            self.get_logger().error("관절 궤적 생성에 실패했습니다.")

    def publish_theta(self, theta):
        """
        계산된 목표 관절각(θ) 명령을 publish합니다.
        주의: 계산 결과가 [0, 0, 0, 0]이면 명령 전송을 생략합니다.
        """
        if np.allclose(theta, np.zeros(4)):
            self.get_logger().warn("계산된 theta가 모두 0입니다. 명령 전송을 생략합니다.")
            return

        msg = Float32MultiArray()
        msg.data = theta.tolist()
        self.theta_pub.publish(msg)
        self.get_logger().info("θ 명령 발행: {}".format(theta))

    def timer_callback(self):
        """
        10ms 주기로 실행되며, 현재 관절 상태와 목표 θ값의 오차를 비교하여
        목표에 도달한 것으로 판단되면 다음 경로점의 θ 명령을 전송합니다.
        """
        if self.executing_trajectory and self.trajectory_joint is not None:
            target_theta = self.trajectory_joint[self.traj_index]
            if target_theta is None:
                self.get_logger().error("경로의 {}번째 점에서 유효하지 않은 θ 값 발생".format(self.traj_index))
                return
            
            # 현재 관절 상태와 목표 θ 사이 오차 계산
            error = np.abs(self.current_joint_angles - target_theta)
            if np.all(error < self.eps):
                # 목표 θ에 도달한 것으로 판단하면 다음 경로점으로 전환
                if self.traj_index < len(self.trajectory_joint) - 1:
                    self.traj_index += 1
                    next_theta = self.trajectory_joint[self.traj_index]
                    if next_theta is not None:
                        self.publish_theta(next_theta)
                    else:
                        self.get_logger().error("경로의 {}번째 점의 θ 값이 유효하지 않습니다.".format(self.traj_index))
                else:
                    # 마지막 경로점에 도달하면 경로 실행 완료 처리
                    self.executing_trajectory = False
                    self.get_logger().info("경로 실행 완료")
            # 아직 목표에 도달하지 않은 경우 별도 동작 없음

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
