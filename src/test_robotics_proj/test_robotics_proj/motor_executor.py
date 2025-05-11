#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np
import threading

OFFSET = np.pi

def theta2abs_ax(theta):
    if theta >= OFFSET:
        theta = OFFSET
    elif theta <= -OFFSET:
        theta = -OFFSET

    abs_angle = (theta / (2 * np.pi)) * 1024 + 512
    return abs_angle

class MotorExecutor(Node):
    def __init__(self):
        super().__init__('motor_executor')
        
        self.qs_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_cal_qs',
            self.qs_callback,
            10
        )
        
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics_goal_position',
            10
        )
        
        self.abs_motor_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_datahub_theta',
            self.datahub_motor_callback,
            10
        )
        self.theta1, self.theta2, self.theta3, self.theta4 = None, None, None, None
    
        self.lock = threading.Lock()
        
    def datahub_motor_callback(self, msg):
        with self.lock:
            if len(msg.data) == 0 or len(msg.data) != 4:
                return
            
            self.theta1, self.theta2, self.theta3, self.theta4 = msg.data
            self.flag = True
        
    def qs_callback(self, msg=Float32MultiArray):
        with self.lock:
            
            while self.flag:
            
                rows = msg.layout.dim[0].size
                cols = msg.layout.dim[1].size
                
                q_list = np.array(msg.data, dtype=float).reshape(rows, cols)
                
                # print(q_list)
                
                qs_msg = Float32MultiArray()
                tol = 2
                
                for q in q_list:
                    qs = [round(theta2abs_ax(self.theta1),2),
                                round(theta2abs_ax(self.theta2),2),
                                round(theta2abs_ax(self.theta3),2),
                                round(theta2abs_ax(self.theta4),2)]
                    qs_msg.data = qs
                    self.motor_pub.publish(qs_msg)
                    print(f"목표 값 publish : {qs_msg.data}")
                    
                    while rclpy.ok():
                        rclpy.spin_once(self, timeout_sec=0.1)
                        if None in (self.theta1, self.theta2, self.theta3, self.theta4):
                            continue
                        
                        curr = np.array([self.theta1, self.theta2, self.theta3, self.theta4])
                        err = np.abs(curr - q)
                        if np.all(err < tol):
                            # print(f"도달 완료 : {curr}")
                            break
                            
                        qs_msg.data = qs
                        self.motor_pub.publish(qs_msg)
                        
                print('END!')
                self.flag = False
        
def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()