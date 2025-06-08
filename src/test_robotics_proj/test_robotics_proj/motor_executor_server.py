#!/usr/bin/env python3

import time, os
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor
from std_msgs.msg import Float32MultiArray, Float32
from test_robotics_proj.Inverse_Kinematics import inverse_kinematics
from test_robotics_proj.Trajectory_Planner import *

class MotorExecutorServer(Node):
    def __init__(self):
        super().__init__('motor_executor_server')
        self.lock = threading.Lock()
        
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/degree/target_theta',
            10
        )
        
        self.present_x, self.present_y, self.present_z = None, None, None
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/coordinate/position',
            self.position_callback,
            10
        )   
        
        # --- joint 5까지 모두 받도록 수정
        self.present_thetas = [None] * 5
        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/radian/theta',
            self.angle_callback,
            10
        )
        
        self.motor_speed_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/abs/speed',
            10
        )
        self.srv = self.create_service(MotorExecutor, 'motor_executor', 
                                       self.service_response_callback)
        
        self.timer_period = 0.5
        self.create_timer(self.timer_period, self.timer_callback)

    def angle_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 5:
                return
            self.present_thetas = list(msg.data)
            
    def position_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 3:
                return
            self.present_x = msg.data[0]
            self.present_y = msg.data[1]
            self.present_z = msg.data[2]
    
    def timer_callback(self):
        pass
        
    def service_response_callback(self, request, response):
        with self.lock:
            try:
                start_point = [self.present_x, self.present_y, self.present_z]
                end_point= [request.x, request.y, request.z]
                grab = request.grab
                radius = request.r
                task = request.task
                
                print(f'요청된 Center 목표 좌표: x={end_point[0]}, y={end_point[1]}, z={end_point[2]}')
                print(f'요청된 작업: {task}, 반경: {radius}')
                
                time.sleep(0.2)
                
                if task == 'circle':
                    print(' Start to Draw Circle ')
                    self.motor_speed_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0]))
                    
                    present_radian = np.deg2rad(self.present_thetas)
                    # --- joint5까지 모두 포함한 initial_q_full
                    q_now = present_radian if None not in present_radian else [0,0,0,0,0]
                    path  = CircleFullPathPlanner(
                        start_point=start_point, 
                        center_point=end_point, 
                        radius=radius, 
                        z_circle=30, 
                        num_move=500, 
                        num_circle=1000
                    ).plan()
                    
                    q_matrix = inverse_kinematics(path, initial_q_full=[0] + list(q_now)) # [0]은 base(joint0)
                    q_matrix = np.array(q_matrix)
                    q_matrix = np.rad2deg(q_matrix)

                    # 5) joint-level 명령 발행 (joint5까지 모두)
                    q_msg = Float32MultiArray()
                    for idx, q in enumerate(q_matrix):
                        # 각도+펄스 프로토콜 일치 필요시 변환 적용
                        # q[4]가 펜/그리퍼라면 theta_5 적용(아니면 q[4] 그대로)
                        q_list = q.tolist()
                        q_msg.data = q_list
                        self.motor_pub.publish(q_msg)
                        time.sleep(0.01)
                        
                    response.success = True
                    self.get_logger().info('플래닝 및 IK 성공')
                
            except Exception as e:
                self.get_logger().error(f'플래닝/IK 실패: {e}')
                response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutorServer()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
