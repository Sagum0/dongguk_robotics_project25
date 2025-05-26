#!/usr/bin/env python3

import time, os
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor
from std_msgs.msg import Float32MultiArray
from test_robotics_proj.Inverse_Kinematics import inverse_kinematics
from test_robotics_proj.Trajectory_Planner import TrajectoryPlanner

def theta2abs_ax(theta):
    abs_angle = (theta / (2 * np.pi)) * 1024 + 512
    return abs_angle

def get_it():
    return int(1955)

def let_it():
    return int(2354)

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
        
        self.present_th1, self.present_th2, self.present_th3, self.present_th4 = None, None, None, None
        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/radian/theta',
            self.angle_callback,
            10
        )
        
        self.srv = self.create_service(MotorExecutor, 'motor_executor', 
                                       self.service_response_callback)
        
        self.timer_period = 0.5
        self.create_timer(self.timer_period, self.timer_callback)

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
                
                # os.system('clear')
                
                print(f'요청된 목표 좌표: x={end_point[0]}, y={end_point[1]}, z={end_point[2]}')
                print(f'요청된 작업: {task}, 반경: {radius}')
                
                time.sleep(1)
                
                if grab:
                    theta_5 = 1955.0
                else:
                    theta_5 = 2354.0
                
                if task == 'move':
                    print(' Gripper가 이동합니다. ')
                    planner = TrajectoryPlanner(start_point=start_point, end_point=end_point, num_points=200)
                    path     = planner.plan()                         # N×3

                    q_matrix = inverse_kinematics(path, initial_q_full=
                        [0, self.present_th1, self.present_th2, self.present_th3, self.present_th4]
                    )
                    
                    q_matrix = np.array(q_matrix)
                    q_matrix = np.rad2deg(q_matrix)

                    # 6) 시간 스케줄 기반 joint‐level 명령 발행
                    q_msg    = Float32MultiArray()
                    for idx, q in enumerate(q_matrix):
                        
                        # 6-2) 목표 각도 변환 및 퍼블리시
                        q_msg.data = [q[0], q[1], q[2], q[3], float(theta_5)]
                        self.motor_pub.publish(q_msg)
                        
                        time.sleep(0.05)
                        
                    response.success = True
                    self.get_logger().info('플래닝 및 IK 성공')
                    
                elif task == 'pick':
                    print(' Gripper를 열거나 닫습니다. ')
                    
                    q_msg    = Float32MultiArray()
                    q_msg.data = [
                            np.rad2deg(self.present_th1), 
                            np.rad2deg(self.present_th2), 
                            np.rad2deg(self.present_th3), 
                            np.rad2deg(self.present_th4), 
                            theta_5
                        ]
                    self.motor_pub.publish(q_msg)
                    
                    response.success = True
                    self.get_logger().info('잡기 성공')
                
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