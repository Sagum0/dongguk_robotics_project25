#!/usr/bin/env python3

import rclpy
import threading
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from test_robotics_proj.Inverse_Kinematics import inverse_k
from test_robotics_proj.Trajectory_Planner import TrajectoryPlanner

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
        
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_datahub_position',
            self.position_callback,
            10
        )   
        self.position_flag = False
        self.present_x, self.present_y, self.present_z = None, None, None
             
        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_datahub_theta',
            self.angle_callback,
            10
        )
        self.present_th1, self.present_th2, self.present_th3, self.present_th4 = None, None, None, None
        self.angle_flag = False
        
        self.lock = threading.Lock()
        
    def angle_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 4:
                self.angle_flag = False
                return
            
            self.present_th1 = msg.data[0]
            self.present_th2 = msg.data[1]
            self.present_th3 = msg.data[2]
            self.present_th4 = msg.data[3]
            
            self.angle_flag = True
        
    def position_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 3:
                self.position_flag = False
                return
            
            self.present_x = msg.data[0]
            self.present_y = msg.data[1]
            self.present_z = msg.data[2]
            
            self.position_flag = True
            
    def coordinate_callback(self, msg=Float32MultiArray):
        with self.lock:
            
            print(f"angle : {self.angle_flag}")
            print(f"Position : {self.position_flag}")
            
            if self.angle_flag or self.position_flag:
                return
            
            if len(msg.data) == 3:
                print(self.position_flag, self.angle_flag)
                
                start_point = [self.present_x, self.present_y, self.present_z]
                print(start_point)
                end_point = msg.data
                
                planner = TrajectoryPlanner(start_point=start_point, end_point=end_point, num_points=100)
                path    = planner.plan()
                
                qs = inverse_k(path)
                q_msg = Float32MultiArray()
                
                tol = 0.1
                
                for q in qs:
                    q_list = [theta2abs_ax(q[0]), theta2abs_ax(q[1]), theta2abs_ax(q[2]), theta2abs_ax(q[3])]
                    q_msg.data = q_list
                    self.motor_pub.publish(q_msg)
                    print(f" Target Angle Published : {q_msg.data}")
                    
                    while rclpy.ok():
                        rclpy.spin_once(self, timeout_sec=0.1)
                        current = np.array([self.present_th1, self.present_th2, self.present_th3, self.present_th4])
                        err = np.abs(current - q)
                        
                        if np.all(err < tol):
                            break
                        
                        self.motor_pub.publish(q_msg)
                        
                print(" All q list is Published! ")

def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()