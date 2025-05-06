#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from robotics_interfaces.msg import TrajectoryPoints

import os
import threading
import numpy as np
from scipy.interpolate import BSpline

class TrajectoryPlanner:
    def __init__(self, start_point, end_point, num_points):
        self.start_point = np.array(start_point)
        self.end_point = np.array(end_point)
        self.num_points = num_points
        
    def generate_trajectory(self):
        dp = self.end_point - self.start_point
        
        t = np.linspace(0.0, 1.0, self.num_points)
        blend = 10*t**3 - 15*t**4 + 6*t**5
        
        path = self.start_point + np.outer(blend, dp)
        
        return path
    
    def bspline(self, path, degree=3, num_points=100):
        N = len(path)
        k = degree
        
        if N <= k:
            raise ValueError("Number of control points must be greater than degree.")
        
        interior = N - (k+1)
        inner_knots = np.linspace(0, 1, interior+2)[1:-1] if interior > 0 else []
        knots = np.concatenate((np.zeros(k+1), inner_knots, np.ones(k+1)))
        
        spline = BSpline(knots, path, k, axis=0)
        u = np.linspace(0, 1, num_points)
        
        return spline(u)
    
    def plan(self):
        poly_path = self.generate_trajectory()
        smoothed_path = self.bspline(poly_path)
        
        return smoothed_path
    
class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')
        self.get_logger().info("Trajectory Planner Node Initialized")
        
        self.publisher = self.create_publisher(TrajectoryPoints, 'trajectory_points', 10)
        
        self.timer = self.create_timer(1, self.timer_callback)
        
    def timer_callback(self):
        start_point = [0, 0, 0]
        end_point = [0.5, 0.8, 0.15]
        num_points = 100
        
        planner = TrajectoryPlanner(start_point, end_point, num_points)
        path = planner.plan()
        
        msg = TrajectoryPoints()
        for x, y, z in path:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            msg.points.append(p)
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
