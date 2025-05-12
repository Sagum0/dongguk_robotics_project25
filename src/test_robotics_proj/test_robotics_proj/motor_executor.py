#!/usr/bin/env python3

import rclpy
import threading
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from test_robotics_proj.Inverse_Kinematics import inverse_k
from test_robotics_proj.Trajectory_Planner import TrajectoryPlanner
import time

def theta2abs_ax(theta):
    abs_angle = (theta / (2 * np.pi)) * 1024 + 512
    return abs_angle

def time_scaling_trap(num_points, T, accel_ratio=0.2):
    t  = np.linspace(0, T, num_points)
    Ta = accel_ratio * T
    Tc = T - 2*Ta
    s  = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < Ta:
            # 가속
            s[i] = 0.5*(ti/Ta)**2 * (Ta/T)
        elif ti < Ta + Tc:
            # 등속
            s[i] = (ti - 0.5*Ta) / T
        else:
            # 감속
            dt   = T - ti
            s[i] = 1 - 0.5*(dt/Ta)**2 * (Ta/T)

    return s

class MotorExecutor(Node):
    def __init__(self):
        super().__init__('motor_executor')
        
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics_goal_position',
            10
        )
        
        self.coordinate_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_goal_coordinate',
            self.coordinate_callback,
            10
        )
        self.end_x, self.end_y, self.end_z = None, None, None
        self.flag = False
        
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_datahub_position',
            self.position_callback,
            10
        )   
        self.present_x, self.present_y, self.present_z = None, None, None
             
        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_datahub_theta',
            self.angle_callback,
            10
        )
        self.present_th1, self.present_th2, self.present_th3, self.present_th4 = None, None, None, None
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.lock = threading.Lock()
        
    def angle_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 4:
                return
            
            self.present_th1 = msg.data[0]
            self.present_th2 = msg.data[1]
            self.present_th3 = msg.data[2]
            self.present_th4 = msg.data[3]
        
    def position_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 3:
                return
            
            self.present_x = msg.data[0]
            self.present_y = msg.data[1]
            self.present_z = msg.data[2]
            
    def coordinate_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 3:
                self.flag = False
                return
            
            self.end_x = msg.data[0]
            self.end_y = msg.data[1]
            self.end_z = msg.data[2]
            
            self.flag = True
        
    def timer_callback(self):
        with self.lock:
            if not self.flag:
                return

            # 1) 시작/끝 좌표 취득
            start_point = [self.present_x, self.present_y, self.present_z]
            end_point   = [self.end_x,    self.end_y,    self.end_z   ]

            # 2) 공간 경로 생성 (폴리+BSpline)
            planner = TrajectoryPlanner(start_point=start_point, end_point=end_point, num_points=400)
            path     = planner.plan()                         # N×3

            # 3) 시간 파라미터화
            T        = 5.0                                   # 총 이동시간 (s)
            s_tbl    = np.linspace(0, 1, len(path))          # 원본 파라미터
            s_time   = time_scaling_trap(len(path), T, accel_ratio=0.1)
            t_array  = s_time * T                            # 각 점의 발행 시각

            # 4) 시간 파라미터에 맞춰 XYZ 재샘플
            path_time = np.vstack([
                np.interp(s_time, s_tbl, path[:, dim])
                for dim in range(3)
            ]).T                                              # N×3

            # 5) IK → q_matrix (N×4)
            q_matrix = inverse_k(path_time)

            # 6) 시간 스케줄 기반 joint‐level 명령 발행
            q_msg    = Float32MultiArray()
            start_t  = time.time()
            for idx, q in enumerate(q_matrix):
                # 6-1) 다음 발행 시각까지 대기
                elapsed = time.time() - start_t
                wait    = t_array[idx] - elapsed
                if wait > 0:
                    time.sleep(wait)

                # 6-2) 목표 각도 변환 및 퍼블리시
                q_msg.data = [
                    theta2abs_ax(q[0]),
                    theta2abs_ax(q[1]),
                    theta2abs_ax(q[2]),
                    theta2abs_ax(q[3]),
                ]
                self.motor_pub.publish(q_msg)

            # 7) 다음 명령 대기를 위해 flag 클리어
            self.flag = False
            self.get_logger().info("All trajectory points published on schedule.")

            
def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()