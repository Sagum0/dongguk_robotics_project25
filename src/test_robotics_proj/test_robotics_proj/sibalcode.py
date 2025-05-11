#!/usr/bin/env python3

import rclpy
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


start_point = [0, 0, 390]
end_point = [0, 100, 200]

arm_chain = Chain(name='4DOF_arm', links=[
    OriginLink(),  # index 0: 베이스 (fixed)
    DHLink(d=0.1075, a=0.0,    alpha=np.pi/2, theta=0),
    DHLink(d=0.0,    a=0.0985, alpha=0.0,     theta=np.pi/2),  # θ₂ offset
    DHLink(d=0.0,    a=0.0985, alpha=0.0,     theta=0),
    DHLink(d=0.0,    a=0.0845, alpha=0.0,     theta=0),
])



arm_chain.active_links_mask[0] = False

class InverseKinematics:
    def __init__(self, start_point, end_point):
        self.start_point = start_point
        self.end_point = end_point
        
    def do_ik(self):
        