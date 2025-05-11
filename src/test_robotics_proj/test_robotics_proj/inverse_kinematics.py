#!/usr/bin/env python3

import rclpy
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from test_robotics_proj.TP import *


class Ikpy_maker:
    def __init__(self, start, end):
        self.arm_chain = Chain(name='4DOF_arm', links=[
            OriginLink(),  # index 0: 베이스 (fixed)
            DHLink(d=0.1075, a=0.0,    alpha=np.pi/2, theta=0),
            DHLink(d=0.0,    a=0.0985, alpha=0.0,     theta=np.pi/2),
            DHLink(d=0.0,    a=0.0985, alpha=0.0,     theta=0),
            DHLink(d=0.0,    a=0.0845, alpha=0.0,     theta=0),
        ])
        self.start = start
        self.end   = end

    def do_ik(self):
        self.arm_chain.active_links_mask[0] = False
        
        planner     = TrajectoryPlanner(start_point=self.start, end_point=self.end, num_points=50)
        path_points = planner.plan()

        prev_q = np.zeros(len(self.arm_chain.links))
        
        q_list = []
        
        first_point = path_points[0]
        q_full = self.arm_chain.inverse_kinematics(
            target_position = first_point,
            initial_position = prev_q
        )
        
        q_list.append(q_full[1:])
        prev_q = q_full
        
        for p in path_points[1:]:
            q_full = self.arm_chain.inverse_kinematics(
                target_position = p,
                initial_position = prev_q
            )
            
            q_list.append(q_full[1:])
            prev_q = q_full
            
        q_matrix = np.array(q_list)
        
        return q_matrix
        
class InverseKinmaticsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.get_logger().info(" Inverse Kinematics Ready ")
        
        self.present_position_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_datahub_position',
            self.present_position_callback,
            10
        )
        self.start = []
        
        self.coordinate_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics_goal_coordinate',
            self.coordinate_callback,
            10
        )
        
        self.x, self.y, self.z = None, None, None
        
        self.cal_q_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics_cal_qs',
            10
        )
        
    def present_position_callback(self, msg):
        self.start = msg.data
        
    def coordinate_callback(self, msg):
        print(msg.data)
        qs_msg = Float32MultiArray()
        
        if len(self.start) == 0:
            return
                
        if self.x is None and self.y is None and self.z is None:
            if len(msg.data) == 3:
                self.x, self.y, self.z = msg.data[0], msg.data[1], msg.data[2]
                end = [self.x, self.y, self.z]
                
                ikpy = Ikpy_maker(self.start, end)
                qs = ikpy.do_ik()
                
                print(qs)
                
                rows, cols = qs.shape
                qs_msg.layout.dim = [
                    MultiArrayDimension(label='points', size=rows, stride=rows*cols),
                    MultiArrayDimension(label='joints', size=cols, stride=cols)
                ]
                qs_msg.layout.data_offset = 0
                qs_msg.data = qs.flatten().tolist()
                
                self.cal_q_pub.publish(qs_msg)
                
                self.x, self.y, self.z = None, None, None
            else:
                self.get_logger().warn(' Wrong Data Length! ')
            
def main(args=None):
    rclpy.init(args=args)
    node = InverseKinmaticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()