#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor
import numpy as np

# 입력 받을 변수 mm 단위

L1 = 200
BOX_TH = 30

L2 = 150
TOP_TH = -45

height_offset = -10.0

# ================

PICK_OFFSET = 30.0
BOX_OPEN_DISTANCE = 50.0  # 서랍 열기 거리
L1_OPEN = L1 - BOX_OPEN_DISTANCE  # 서랍 열기 후 L1 길이

BOX_RAD = np.deg2rad(BOX_TH)
TOP_RAD = np.deg2rad(TOP_TH)

BOX_SIN = np.sin(BOX_RAD)
BOX_COS = np.cos(BOX_RAD)

TOP_SIN = np.sin(TOP_RAD)
TOP_COS = np.cos(TOP_RAD)

TOP_1_DIS = L2 + 62.5
TOP_1 = [TOP_1_DIS * TOP_COS, TOP_1_DIS * TOP_SIN, 157.5 + height_offset]

###

TOP_2_DIS = L2 + 52.5
TOP_2 = [TOP_2_DIS * TOP_COS, TOP_2_DIS * TOP_SIN, 132.5 + height_offset]

TOP_3_DIS = L2 + 82.5
TOP_3 = [TOP_3_DIS * TOP_COS, TOP_3_DIS * TOP_SIN, 132.5 + height_offset]

###

TOP_4_DIS = L2 + 37.5
TOP_4 = [TOP_4_DIS * TOP_COS, TOP_4_DIS * TOP_SIN, 107.5 + height_offset]

TOP_5_DIS = L2 + 62.5
TOP_5 = [TOP_5_DIS * TOP_COS, TOP_5_DIS * TOP_SIN, 107.5 + height_offset]

TOP_6_DIS = L2 + 97.5
TOP_6 = [TOP_6_DIS * TOP_COS, TOP_6_DIS * TOP_SIN, 107.5 + height_offset]

###

TOP_7_DIS = L2 + 22.5
TOP_7 = [TOP_7_DIS * TOP_COS, TOP_7_DIS * TOP_SIN, 82.5 + height_offset]

TOP_8_DIS = L2 + 52.5
TOP_8 = [TOP_8_DIS * TOP_COS, TOP_8_DIS * TOP_SIN, 82.5 + height_offset]

TOP_9_DIS = L2 + 82.5
TOP_9 = [TOP_9_DIS * TOP_COS, TOP_9_DIS * TOP_SIN, 82.5 + height_offset]

TOP_10_DIS = L2 + 37.5
TOP_10 = [TOP_10_DIS * TOP_COS, TOP_10_DIS * TOP_SIN, 57.5 + height_offset]

STEADY_DIS = L2 - 30.0
STEADY = [STEADY_DIS * TOP_COS, STEADY_DIS * TOP_SIN, 180.0 + height_offset]

# ================

coordinate_list = [
    # 서랍 열기
    [L1 * BOX_COS, L1 * BOX_SIN, 150.0, False, 'move'],
    [L1 * BOX_COS, L1 * BOX_SIN, 30.0, False, 'move'], # 서랍 위치까지 하강
    [L1 * BOX_COS, L1 * BOX_SIN, 30.0, True, 'pick'], # 서랍 집기
    [L1_OPEN * BOX_COS, L1_OPEN * BOX_SIN, 30.0, True, 'move'], # 서랍 위치에서 서랍 열기 위치로 이동
    [L1_OPEN * BOX_COS, L1_OPEN * BOX_SIN, 30.0, False, 'pick'], # 서랍 놓기
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'move'],  # 초기 위치로 이동
    
    # 박스 집으러 이동
    [STEADY[0], STEADY[1], STEADY[2], False, 'move'],  # 안정 위치로 이동
    [TOP_1[0], TOP_1[1], TOP_1[2], False, 'move'],  # 박스 위치로 이동
    [TOP_1[0], TOP_1[1], TOP_1[2], True, 'pick'],  # 박스 집기
    [TOP_1[0], TOP_1[1], TOP_1[2] + PICK_OFFSET, True, 'move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'move'],  # 안정 위치로 이동
    
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

        x, y, z, grab, task = coordinate_list[self.idx]
        req = MotorExecutor.Request()
        req.x = x
        req.y = y
        req.z = z
        req.task = task
        req.grab = grab

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



