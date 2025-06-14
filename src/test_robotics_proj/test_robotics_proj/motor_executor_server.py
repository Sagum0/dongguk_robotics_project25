#!/usr/bin/env python3

import time, os
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Float32
from test_robotics_proj.Inverse_Kinematics import inverse_kinematics
from test_robotics_proj.Trajectory_Planner import *

C_START_OFFSET = 60.0
C_DRAW_OFFSET  = 20.0

class MotorExecutorServer(Node):
    def __init__(self):
        super().__init__('motor_executor_server')
        self.sub_lock = threading.Lock()
        self.lock = threading.Lock()
        
        qos = QoSProfile(depth=10)
        
        self.cg_sub = ReentrantCallbackGroup()        # 구독용 그룹
        self.cg_srv = MutuallyExclusiveCallbackGroup()  # 서비스용 그룹
        
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
            qos_profile=qos,  
            callback_group=self.cg_sub
        )   
        
        # --- joint 5까지 모두 받도록 수정
        self.present_thetas = [None] * 5
        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/radian/theta',
            self.angle_callback,
            qos_profile=qos,  
            callback_group=self.cg_sub
        )
        
        self.motor_speed_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/abs/speed',
            10
        )
        self.srv = self.create_service(MotorExecutor, 'motor_executor', 
                                       self.service_response_callback, callback_group=self.cg_srv)
        
        self.timer_period = 0.5
        self.create_timer(self.timer_period, self.timer_callback)

    def angle_callback(self, msg=Float32MultiArray):
        with self.sub_lock:
            if len(msg.data) != 5:
                return
            self.present_thetas = list(msg.data)
            # print(f"Callback :  {self.present_thetas}")
            
    def position_callback(self, msg=Float32MultiArray):
        with self.sub_lock:
            if len(msg.data) != 3:
                return
            self.present_x = msg.data[0]
            self.present_y = msg.data[1]
            self.present_z = msg.data[2]
    
    def timer_callback(self):
        pass
        
    def service_response_callback(self, request, response):
        try:
            start_point = [self.present_x, self.present_y, self.present_z]
            grab = request.grab
            radius = request.r
            task = request.task
            
            time.sleep(0.2)
            
            if task == 'circle':
                print(' Start to Draw Circle ')
                
                print(" ")
                print(" Step #1 : 중심 좌표로 이동 ")
                print(" ")
                
                initial_q_full = np.deg2rad(np.array([0] + self.present_thetas, dtype=float))
                print(f"step    :  {initial_q_full}")
            
                start_point  = [self.present_x, self.present_y, self.present_z] 
                center_point = [request.x, request.y, request.z]
                
                path_1 = np.vstack((start_point, center_point))
                step_1_q_matrix = inverse_kinematics(path_1, 
                                                        initial_q_full)
                
                self.motor_speed_pub.publish(Float32MultiArray(data=[70.0, 70.0, 70.0, 70.0, 90.0]))
                
                step_1_q_matrix = np.array(step_1_q_matrix)
                step_1_q_matrix = np.rad2deg(step_1_q_matrix)
                
                step_1_q_msg    = Float32MultiArray()
                for idx, q in enumerate(step_1_q_matrix):
                    
                    # 6-2) 목표 각도 변환 및 퍼블리시
                    step_1_q_msg.data = [q[0], q[1], q[2], q[3], q[4]]
                    self.motor_pub.publish(step_1_q_msg)
                    
                time.sleep(2.0)
                
                print(" ")
                print(" Step #2 : 원그리기로 이동 ")
                print(" ")
                

                last_q = step_1_q_matrix[-1]                # numpy 1D array
                initial_q_full = np.deg2rad(
                    np.concatenate(([0], last_q))          # dummy 0 필요 없으면 just last_q
                )
                
                circle_path = generate_circle_path(request.x, request.y, C_DRAW_OFFSET, radius, num_circle = 200) 
                
                c_start_x, c_start_y, _ = circle_path[0]
                c_start = (c_start_x, c_start_y, C_START_OFFSET)
                

                start_point  = [self.present_x, self.present_y, self.present_z]
                path_2 = np.vstack((start_point, c_start))
                step_2_q_matrix = inverse_kinematics(path_2, 
                                                        initial_q_full)
                
                self.motor_speed_pub.publish(Float32MultiArray(data=[70.0, 70.0, 70.0, 70.0, 70.0]))
                
                step_2_q_matrix = np.array(step_2_q_matrix)
                step_2_q_matrix = np.rad2deg(step_2_q_matrix)
                
                step_2_q_msg    = Float32MultiArray()
                for idx, q in enumerate(step_2_q_matrix):
                    
                    # 6-2) 목표 각도 변환 및 퍼블리시
                    step_2_q_msg.data = [q[0], q[1], q[2], q[3], q[4]]
                    self.motor_pub.publish(step_2_q_msg)
                    
                time.sleep(2.0)
                
                print(" ")
                print(" Step #3 : 하강 ")
                print(" ")
                
                last_q = step_2_q_matrix[-1]                # numpy 1D array
                initial_q_full = np.deg2rad(
                    np.concatenate(([0], last_q))          # dummy 0 필요 없으면 just last_q
                )
                
                c_start = (c_start_x, c_start_y, C_DRAW_OFFSET)
                
                start_point  = [self.present_x, self.present_y, self.present_z]
                    
                path_3 = np.vstack((start_point, c_start))
                step_3_q_matrix = inverse_kinematics(path_3, 
                                                        initial_q_full)
                
                self.motor_speed_pub.publish(Float32MultiArray(data=[5.0, 5.0, 5.0, 5.0, 10.0]))
                
                step_3_q_matrix = np.array(step_3_q_matrix)
                step_3_q_matrix = np.rad2deg(step_3_q_matrix)
                
                step_3_q_msg    = Float32MultiArray()
                for idx, q in enumerate(step_3_q_matrix):
                    
                    # 6-2) 목표 각도 변환 및 퍼블리시
                    step_3_q_msg.data = [q[0], q[1], q[2], q[3], q[4]]
                    self.motor_pub.publish(step_3_q_msg)
                
                time.sleep(3.0)
                    
                print(" ")
                print(" Step #4 : 원 그리기 ")
                print(" ")
                
                self.motor_speed_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0]))
                
                last_q = step_3_q_matrix[-1]                # numpy 1D array
                initial_q_full = np.deg2rad(
                    np.concatenate(([0], last_q))          # dummy 0 필요 없으면 just last_q
                )

                step_4_q_matrix = inverse_kinematics(circle_path, initial_q_full)
                step_4_q_matrix = np.array(step_4_q_matrix)
                step_4_q_matrix = np.rad2deg(step_4_q_matrix)

                step_4_q_msg = Float32MultiArray()
                for idx, q in enumerate(step_4_q_matrix):
                    q_list = q.tolist()
                    step_4_q_msg.data = q_list
                    self.motor_pub.publish(step_4_q_msg)
                    time.sleep(0.1)
                    
                time.sleep(5.0)
                
                print(" ")
                print(" Step #5 : 상승 ")
                print(" ")
                
                last_q = step_4_q_matrix[-1]                # numpy 1D array
                initial_q_full = np.deg2rad(
                    np.concatenate(([0], last_q))          # dummy 0 필요 없으면 just last_q
                )
                
                c_last = circle_path[-1]
                start_point  = [self.present_x, self.present_y, self.present_z]
                up_point     = [c_last[0], c_last[1], C_START_OFFSET]

                time.sleep(1.0)
                    
                path_5 = np.vstack((start_point, up_point))
                step_5_q_matrix = inverse_kinematics(path_5, 
                                                        initial_q_full)
                
                self.motor_speed_pub.publish(Float32MultiArray(data=[10.0, 10.0, 10.0, 10.0, 10.0]))
                
                step_5_q_matrix = np.array(step_5_q_matrix)
                step_5_q_matrix = np.rad2deg(step_5_q_matrix)
                
                step_5_q_msg    = Float32MultiArray()
                for idx, q in enumerate(step_5_q_matrix):
                    
                    # 6-2) 목표 각도 변환 및 퍼블리시
                    step_5_q_msg.data = [q[0], q[1], q[2], q[3], q[4]]
                    self.motor_pub.publish(step_5_q_msg)
                
                time.sleep(1.0)
                
                response.success = True
                self.get_logger().info('플래닝 및 IK 성공')
            
        except Exception as e:
            self.get_logger().error(f'플래닝/IK 실패: {e}')
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutorServer()

    # 3) 멀티쓰레드 스피너 사용
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()