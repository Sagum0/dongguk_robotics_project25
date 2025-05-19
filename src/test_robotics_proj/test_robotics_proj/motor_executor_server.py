#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor

class MotorExecutorServer(Node):
    def __init__(self):
        super().__init__('motor_executor_server')
        self.srv = self.create_service(MotorExecutor, 'motor_executor', self.executor_callback)
        
    def executor_callback(self, request, response):
        self.x, self.y, self.z = request.x, request.y, request.z
        self.task = response.task
        self.r = response.r
        
        self.get_logger().info(f' Goal Coordinate x : {self.x:.2f}, y : {self.x:.2f}, z : {self.x:.2f}')
        
        # --- 동작이 완료가 되면 ---
        
        response.success = True
        return response
    
    
def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutorServer()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=='__main__':
    main()