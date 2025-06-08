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

D1 = 120.75
A2 = 125.25
A3 = 110.25
A4 = 98.5
A5 = 45.0
ALPHA_1 = np.pi / 2
ALPHA_5 = np.pi / 2

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

def forward_kinematics(theta1, theta2, theta3, theta4, theta5):
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
    
    # 3 -> 4
    T34 = dh_transform(theta4, 0.0, A4, 0.0)
    T04 = T03 @ T34
    p4 = T04[:3, 3]
    points.append(p4)
    
    # 4 -> 5 (펜/엔드이펙터)
    T45 = dh_transform(theta5, 0.0, A5, ALPHA_5)
    T05 = T04 @ T45
    p5 = T05[:3, 3]
    points.append(p5)
    
    return T05, points

def theta_to_pulse(theta, deg_min=-180.0, deg_max=180.0, resolution=4096):
    theta = np.asarray(theta)
    theta_clamped = np.clip(theta, deg_min, deg_max)
    pulse = (theta_clamped - deg_min) / (deg_max - deg_min) * (resolution - 1)
    return pulse.astype(float) 

def pulse_to_theta(pulse, deg_min=-180.0, deg_max=180.0, resolution=4096):
    pulse_arr = np.asarray(pulse)
    pulse_clamped = np.clip(pulse_arr, 0, resolution - 1)
    theta_deg = pulse_clamped / (resolution - 1) * (deg_max - deg_min) + deg_min
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
        
        self.lock = threading.Lock()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.data_received = False

        # 5자유도 각도/펄스 변수
        self.abs_angles = np.zeros(5)    # abs_angle_1~5
        self.thetas     = np.zeros(5)    # theta1~5
        self.x, self.y, self.z = 0, 0, 0
    
    def abs_motor_callback(self, msg):
        data = msg.data
        
        os.system('clear')
        
        if len(data) < 5:
            print(' ')
            self.get_logger().error(" Motor 절대 각도가 수신되지 않습니다. ")
            print(' ')
            return 
        
        with self.lock:
            self.abs_angles = np.array(data[:5])
            self.thetas = pulse_to_theta(self.abs_angles)
            
            T, _ = forward_kinematics(*self.thetas)
            self.x, self.y, self.z = T[:3, 3]
            
            self.data_received = True
            
            print(" ==================== DATA HUB ==================== ")
            print(" ")
            for i in range(5):
                print(f" Joint #{i+1} : Abs : {self.abs_angles[i]:.1f} RAD : {self.thetas[i]:.3f} // theta #{i+1} : {np.rad2deg(self.thetas[i]):.3f}")
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
            thetas = self.thetas.tolist()
        
        position_msg.data = [x, y, z]
        angle_msg.data = thetas
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
            target_pulse.data = th.tolist()  # 모든 joint pulse를 변환
            self.target_pulse_pub.publish(target_pulse)
        
def main(args=None):
    rclpy.init(args=args)
    node = DataHub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
