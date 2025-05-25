#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor

coordinate_list = [
    [50.0,  50.0, 200.0],
    [100.0, 100.0, 100.0],
    [120.0, 120.0, 50.0],
    [100.0, 100.0, 100.0],
    [100.0, -100.0, 100.0],
    [120.0, -120.0, 50.0],
    # 다음 좌표를 여기에 추가...
]

class MotorExecutorClient(Node):
    def __init__(self):
        super().__init__('motor_executor_client')
        self.client = self.create_client(MotorExecutor, 'motor_executor')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 서버 대기 중...')
        
        self.idx = 0
        self.send_next_goal()

    def send_next_goal(self):
        if self.idx >= len(coordinate_list):
            self.get_logger().info('모든 좌표 전송 완료')
            return

        x, y, z = coordinate_list[self.idx]
        req = MotorExecutor.Request()
        req.x = x
        req.y = y
        req.z = z

        self.get_logger().info(f'[{self.idx}] 목표 좌표 전송: x={x:.2f}, y={y:.2f}, z={z:.2f}')
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'서비스 호출 실패: {e}')
            return

        if res.success:
            self.get_logger().info(f'[{self.idx}] 동작 성공')
            self.idx += 1
            # 다음 좌표 전송
            self.send_next_goal()
        else:
            self.get_logger().warn(f'[{self.idx}] 동작 실패')
            # 재시도하거나 종료 로직 추가 가능

def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutorClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
