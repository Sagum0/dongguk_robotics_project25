#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor

class MotorExecutorClient(Node):
    def __init__(self):
        super().__init__('motor_executor_client')
        self.client = self.create_client(MotorExecutor, 'motor_executor')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' Wait for Server ... ')
        
        self.timer_period = 0.5
        
        self.create_timer(self.timer_period, self.timer_callback)
        
    def timer_callback(self):
    
    
def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutorClient()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=='__main__':
    main()