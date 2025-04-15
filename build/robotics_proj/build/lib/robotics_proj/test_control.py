#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray

import os

D1 = 107.5
A2  = 98.5
A3  = 98.5
A4  = 84.5
ALPHA_1 = np.pi / 2

def dh_transform(theta, d, a, alpha):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

def forward_kinematics(joint_angles):
    # os.system('clear')
    theta1, theta2, theta3, theta4 = joint_angles
    base_point = np.array([0, 0, 0])
    points = [base_point]
    
    # 0 -> 1
    T01 = dh_transform(theta1, D1, 0.0, ALPHA_1)
    p1 = T01[:3, 3]
    points.append(p1)
    
    # 1 -> 2
    T12 = dh_transform(theta2 + np.pi/2, 0.0, A2, 0.0)
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
    
    #print("p1 ", p1)
    #print("p2 ", p2)
    #print("p3 ", p3)
    print("p4 ", p4)
    
    #print("theta1: ", theta1)
    #print("theta2: ", theta2)
    #print("theta3: ", theta3)
    #print("theta4: ", theta4)
    
    return T04, points

def jacobian_numerical(q, eps=1e-6):
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

def inverse_kinematics(target, q_init=np.zeros(4), lam=0.05, iters=100):
    q = q_init.copy()
    for _ in range(iters):
        T, _ = forward_kinematics(q)
        pos = T[:3, 3]
        e = target - pos  # 위치 오차 (mm)
        if np.linalg.norm(e) < 1e-3:
            # 오차가 매우 작으면 수렴했다고 봅니다.
            return q
        # 수치적으로 자코비안을 구합니다.
        J = jacobian_numerical(q)
        # 댐핑 항을 포함한 의사역행렬 계산
        # J_dls = J^T (J J^T + lam^2*I)^(-1)
        JT = J.T
        JJT = J @ JT  # 3x3 행렬
        damped_term = JJT + (lam**2) * np.eye(3)
        J_dls = JT @ np.linalg.inv(damped_term)
        dq = J_dls @ e
        q = q + dq
        print(q)
    # 지정 반복 이내에 수렴하지 않으면 에러 발생
    if np.linalg.norm(e) >= 1e-3:
        raise ValueError("Inverse Kinematics did not converge!")
    
    return q

# ====================================================
# 7. 보간 함수들: Cubic (5차 다항식) 및 Linear (선형) 보간
# ====================================================
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

# ====================================================
# 8. 각 경로점에 대해 역기구학을 풀어 관절각 궤적 생성
# ====================================================
def compute_joint_trajectory(traj):
    """
    생성된 3차원 경로 traj (각 행이 [x,y,z])에 대해,
    각 위치마다 DLS Jacobian IK를 풀어 관절각 벡터를 계산합니다.
    """
    joint_traj = []
    q_current = np.zeros(4)
    for point in traj:
        try:
            # 이전 관절각을 초기 추정치로 사용하면 수렴이 빠릅니다.
            q_current = inverse_kinematics(point, q_init=q_current)
        except ValueError as e:
            print(e)
            q_current = None
        joint_traj.append(q_current)
    return joint_traj

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info("ControlNode 노드 시작")
        
        self.theta_pub = self.create_publisher(Float32MultiArray, '/robotics_goal_position', 10)
        self.create_subscription(Float32MultiArray, '/robotics_present_position', self.present_callback, 10)
        self.create_subscription(Float32MultiArray, '/robotics_target_position', self.goal_callback, 10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.current_joint_angles = np.zeros(4)
        
        # 현재 관절 상태 (초기값: [0,0,0,0])
        self.current_joint_angles = np.zeros(4)
        # 생성된 관절궤적 (θ 값들)와 진행 인덱스
        self.trajectory_joint = None
        self.traj_index = 0
        self.executing_trajectory = False
        
        # 허용 오차 (rad)
        self.eps = 0.05
        
        # 경로 생성을 위한 파라미터 (전체 이동 시간, 프레임 수)
        self.tf = 10  # 전체 이동 시간 (초)
        self.num_frames = int(self.tf / timer_period)  # 타이머 주기에 따른 프레임 수
        
    def present_callback(self, msg):
        data = msg.data
        if len(data) >= 4:
            motor_vals = np.array(data[:4])
            # 모터 기준 512가 0 rad이므로 변환
            self.current_joint_angles = (motor_vals - 512) / 1024 * np.pi
            T, _ = forward_kinematics(self.current_joint_angles)
            self.get_logger().info("현재 관절 상태: {} (x,y,z) = {}".format(
                self.current_joint_angles, T[:3, 3]))
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
        if np.allclose(theta, np.zeros(4)):
            self.get_logger().warn("계산된 theta가 모두 0입니다. 명령 전송을 생략합니다.")
            return

        msg = Float32MultiArray()
        msg.data = theta.tolist()
        self.theta_pub.publish(msg)
        self.get_logger().info("θ 명령 발행: {}".format(theta))
        
    def timer_callback(self):
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