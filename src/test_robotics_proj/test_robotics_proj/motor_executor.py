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
        
        # 초기 자세 1회성 전달
        initial_msg = Float32MultiArray()
        inital_point = [0.0, 0.0, 390.0]
        initial_msg.data = inital_point
        self.motor_pub.publish(initial_msg)
        # =================
        
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
            planner = TrajectoryPlanner(start_point=start_point, end_point=end_point, num_points=10)
            path     = planner.plan()                         # N×3

            q_matrix = inverse_k(path)

            # 6) 시간 스케줄 기반 joint‐level 명령 발행
            q_msg    = Float32MultiArray()
            for idx, q in enumerate(q_matrix):

                # 6-2) 목표 각도 변환 및 퍼블리시
                q_msg.data = [
                    theta2abs_ax(q[0]),
                    theta2abs_ax(q[1]),
                    theta2abs_ax(q[2]),
                    theta2abs_ax(q[3]),
                ]
                self.motor_pub.publish(q_msg)
                
                time.sleep(0.005)

            # 7) 다음 명령 대기를 위해 flag 클리어
            self.flag = False
            self.get_logger().info("All trajectory points published on schedule.")
 
def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()