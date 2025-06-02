#!/usr/bin/env python3

import time, os
# import threading
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor
from std_msgs.msg import Float32MultiArray, Float32
from test_robotics_proj.Inverse_Kinematics import inverse_kinematics

def get_it():
    return int(1955)

def let_it():
    return int(2354)

def calculate_speed(q_vector):
    """
    q_vector: (4,) 혹은 (3,) 크기로 들어온 joint 각도 중
            [q1, q2, q3] 부분을 사용하여 속도 비례 계산 후 (3,) 크기 리스트/array 반환
    """
    # q_vector 에서 q1, q2, q3만 뽑아서 넘겨줌
    arr = np.array(q_vector[1:4], dtype=float)
    
    arr = np.abs(arr)  # 절대값으로 변환
    
    # 최대값과 인덱스 찾기
    max_idx = int(np.argmax(arr))
    max_val = arr[max_idx]

    if max_val == 0:
        speeds = np.full(3, 20.0)
    else:
        k = 100.0 / max_val
        speeds = arr * k
        speeds = np.round(speeds).astype(float)
        speeds[max_idx] = 100.0
        
        speeds = np.where(speeds < 40, 40, speeds)

    return speeds.tolist()

class MotorExecutorServer(Node):
    def __init__(self):
        super().__init__('motor_executor_server')
        # self.lock = threading.Lock()
        
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
        
        self.gripper_pub = self.create_publisher(
            Float32,
            '/robotics/gripper/command',
            10
        )
        
        self.motor_speed_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/speed/pulse_speed',
            10
        )
        
        self.srv = self.create_service(MotorExecutor, 'motor_executor', 
                                       self.service_response_callback)
        
        self.timer_period = 0.5
        self.create_timer(self.timer_period, self.timer_callback)
        self.tol = 15

    def angle_callback(self, msg=Float32MultiArray):
        # with self.lock:
            if len(msg.data) != 4:
                return
            
            self.present_th1 = msg.data[0]
            self.present_th2 = msg.data[1]
            self.present_th3 = msg.data[2]
            self.present_th4 = msg.data[3]
            
    def position_callback(self, msg=Float32MultiArray):
        # with self.lock:
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
            end_point= [request.x, request.y, request.z]
            grab = request.grab
            radius = request.r
            task = request.task
            
            print(f'요청된 목표 좌표: x={end_point[0]}, y={end_point[1]}, z={end_point[2]}')
            print(f'요청된 작업: {task}, 반경: {radius}')
            
            time.sleep(1)
            
            if grab:
                theta_5 = get_it()
            else:
                theta_5 = let_it()
            
            if task == 'move':
                print(' Gripper가 이동합니다. ')

                path  = np.vstack((start_point, end_point))
                
                q_matrix = inverse_kinematics(path, 
                                                initial_q_full=[0, self.present_th1, self.present_th2, self.present_th3, self.present_th4])
                
                q_matrix = np.array(q_matrix)
                q_matrix = np.rad2deg(q_matrix)

                # 6) 시간 스케줄 기반 joint‐level 명령 발행
                q_msg    = Float32MultiArray()
                speed_msg = Float32MultiArray()

                for idx, q in enumerate(q_matrix):
                    
                    speeds = calculate_speed(q)
                    
                    speed_msg.data = [100.0, float(speeds[0]), float(speeds[1]), float(speeds[2])]
                    print(f'발행할 속도: {speed_msg.data}')
                    self.motor_speed_pub.publish(speed_msg)
                    time.sleep(0.2)
                            
                    q_msg.data = [q[0], q[1], q[2], q[3], float(theta_5)]
                    self.motor_pub.publish(q_msg)
    
                response.success = True
                self.get_logger().info('플래닝 및 IK 성공')
                
            elif task == 'pick':
                print(' Gripper를 열거나 닫습니다. ')
                
                gripper_msg = Float32()
                gripper_msg.data = float(theta_5)
                self.gripper_pub.publish(gripper_msg)
                time.sleep(1)
                
                response.success = True
                self.get_logger().info('잡기 성공')
            
        except Exception as e:
            self.get_logger().error(f'플래닝/IK 실패: {e}')
            response.success = False

        return response 
    
    
def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutorServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__=='__main__':
    main()