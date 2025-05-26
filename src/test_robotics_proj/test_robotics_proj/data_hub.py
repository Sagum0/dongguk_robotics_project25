#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk import *  # PortHandler, PacketHandler 등 제공

import os
import threading
import numpy as np

timer_period = 0.1 # 1 : 1초

D1 = 107.5
A2 = 98.5
A3 = 98.5
A4 = 84.5
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

def forward_kinematics(theta1, theta2, theta3, theta4):
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
    
    return T04, points

def theta_to_pulse(theta, deg_min=-150.0, deg_max=150.0, resolution=1024):
    """
    라디안 단위 각(theta_rad)을 pulse 값으로 변환 (0 ~ resolution-1)
    - deg_min → 0
    - deg_max → resolution-1
    """                                # 라디안 → 도
    theta_clamped = np.clip(theta, deg_min, deg_max)                # 최소/최대 클램핑
    pulse = (theta_clamped - deg_min) / (deg_max - deg_min) * (resolution - 1)
    return pulse.astype(float) 

def pulse_to_theta(pulse, deg_min=-150.0, deg_max=150.0, resolution=1024):
    """
    pulse 값(0 ~ resolution-1)을 라디안 단위 각도로 변환
    - 0          → deg_min 도
    - resolution-1 → deg_max 도
    """
    pulse_arr = np.asarray(pulse)
    # pulse 범위 클램핑
    pulse_clamped = np.clip(pulse_arr, 0, resolution - 1)
    # 도 단위로 매핑
    theta_deg = pulse_clamped / (resolution - 1) * (deg_max - deg_min) + deg_min
    # 라디안 변환
    return np.deg2rad(theta_deg)

class DataHub(Node):
    def __init__(self):
        super().__init__('data_hub')
        self.get_logger().info(" Data Hub On! ")
        
        self.abs_motor_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/abs/pulse',
            self.abs_motor_callback,
            10
        )
        
        self.theta_angle_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/radian/theta',
            10
        )
        
        self.position_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/coordinate/position',
            10
        )
        
        self.target_pulse_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/abs/target_pulse',
            10
        )
        
        self.target_pulse_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/degree/target_theta',
            self.target_degree2pulse_callback,
            10
        )
        
        self.abs_motor_sub
        
        self.lock = threading.Lock()
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.data_received = False
        
        self.abs_angle_1, self.abs_angle_2, self.abs_angle_3, self.abs_angle_4 = 0, 0, 0, 0
        self.theta1, self.theta2, self.theta3, self.theta4 = 0, 0, 0, 0
        self.x, self.y, self.z = 0, 0, 0
        
    
    def abs_motor_callback(self, msg):
        data = msg.data
        
        os.system('clear')
        
        if len(data) < 4:
            print(' ')
            self.get_logger().error(" Motor 절대 각도가 수신되지 않습니다. ")
            print(' ')
            
            return 
        
        with self.lock:
            self.abs_angle_1, self.abs_angle_2, self.abs_angle_3, self.abs_angle_4 = data
            
            self.theta1 = pulse_to_theta(self.abs_angle_1)
            self.theta2 = pulse_to_theta(self.abs_angle_2)
            self.theta3 = pulse_to_theta(self.abs_angle_3)
            self.theta4 = pulse_to_theta(self.abs_angle_4)
            
            T, _ = forward_kinematics(self.theta1, self.theta2, self.theta3, self.theta4)
            self.x, self.y, self.z = T[:3, 3]
            
            self.data_received = True
            
            print(" ==================== DATA HUB ==================== ")
            print(" ")
            print(f" Joint #1 : Abs : {self.abs_angle_1:.1f} RAD : {self.theta1:.3f} // theta #1 : {np.rad2deg(self.theta1):.3f}")
            print(f" Joint #2 : Abs : {self.abs_angle_2:.1f} RAD : {self.theta2:.3f} // theta #2 : {np.rad2deg(self.theta2):.3f}")
            print(f" Joint #3 : Abs : {self.abs_angle_3:.1f} RAD : {self.theta3:.3f} // theta #3 : {np.rad2deg(self.theta3):.3f}")
            print(f" Joint #4 : Abs : {self.abs_angle_4:.1f} RAD : {self.theta4:.3f} // theta #4 : {np.rad2deg(self.theta4):.3f}")
            print(" ")
            print(f" Position X Y Z : {self.x:.3f} | {self.y:.3f} | {self.z:.3f}")
            print(" ")
            print(" ==================== DATA HUB ==================== ")
          
    def timer_callback(self):
        with self.lock:
            if self.data_received != True:
                self.get_logger().warn(' No INFO to Publish !! ')
                
                return 
            
            position_msg = Float32MultiArray()
            angle_msg = Float32MultiArray()
            
            x, y, z = self.x, self.y, self.z
            theta1, theta2, theta3, theta4 = self.theta1, self.theta2, self.theta3, self.theta4
            
        position_msg.data = [x,y,z]
        
        angle_msg.data = [theta1,theta2,theta3,theta4]
        
        self.position_pub.publish(position_msg)
        self.theta_angle_pub.publish(angle_msg)
        
    def target_degree2pulse_callback(self, msg):
        data = msg.data
        
        if len(data) < 5:
            self.get_logger().error(" Target degree array length is less than 5.")
            return
        with self.lock:
            th = theta_to_pulse(data)
            
            target_pulse = Float32MultiArray()
            target_pulse.data = [th[0], th[1], th[2], th[3], data[4]]
            
            self.target_pulse_pub.publish(target_pulse)
        
def main(args=None):
    rclpy.init(args=args)
    node = DataHub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()