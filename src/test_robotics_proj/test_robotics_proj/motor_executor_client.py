#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor
import numpy as np

# ================
# 입력 받을 변수 mm 단위

L1 = 175
BOX_TH = 60

L2_INPUT = 125
TOP_TH = -45

# ================

L2 = L2_INPUT - 9.5

height_offset = -10.0
HANDLE = L1 - 20.0
PICK_OFFSET = 40.0
BOX_OPEN_DISTANCE = 60.0  # 서랍 열기 거리
L1_OPEN = L1 - BOX_OPEN_DISTANCE - 15.0  # 서랍 열기 후 L1 길이

BOX_RAD = np.deg2rad(BOX_TH)
TOP_RAD = np.deg2rad(TOP_TH)

BOX_SIN = np.sin(BOX_RAD)
BOX_COS = np.cos(BOX_RAD)

TOP_SIN = np.sin(TOP_RAD)
TOP_COS = np.cos(TOP_RAD)

TOP_1_DIS = L2 + 67.5 -3
TOP_1 = [TOP_1_DIS * TOP_COS, TOP_1_DIS * TOP_SIN, 157.5 + height_offset]

###

TOP_2_DIS = L2 + 52.5 -3
TOP_2 = [TOP_2_DIS * TOP_COS, TOP_2_DIS * TOP_SIN, 132.5 + height_offset]

TOP_3_DIS = L2 + 82.5 -3
TOP_3 = [TOP_3_DIS * TOP_COS, TOP_3_DIS * TOP_SIN, 132.5 + height_offset]

###

TOP_4_DIS = L2 + 37.5 -3
TOP_4 = [TOP_4_DIS * TOP_COS, TOP_4_DIS * TOP_SIN, 107.5 + height_offset]

TOP_5_DIS = L2 + 67.5 -3
TOP_5 = [TOP_5_DIS * TOP_COS, TOP_5_DIS * TOP_SIN, 107.5 + height_offset]

TOP_6_DIS = L2 + 97.5 -3
TOP_6 = [TOP_6_DIS * TOP_COS, TOP_6_DIS * TOP_SIN, 107.5 + height_offset]

###

TOP_7_DIS = L2 + 22.5 -3
TOP_7 = [TOP_7_DIS * TOP_COS, TOP_7_DIS * TOP_SIN, 82.5 + height_offset]

TOP_8_DIS = L2 + 52.5 -3
TOP_8 = [TOP_8_DIS * TOP_COS, TOP_8_DIS * TOP_SIN, 82.5 + height_offset]

TOP_9_DIS = L2 + 82.5 -3
TOP_9 = [TOP_9_DIS * TOP_COS, TOP_9_DIS * TOP_SIN, 82.5 + height_offset]

TOP_10_DIS = L2 + 112.5 -3
TOP_10 = [TOP_10_DIS * TOP_COS, TOP_10_DIS * TOP_SIN, 82.5 + height_offset]

STEADY_DIS = L2 - 30.0
STEADY = [STEADY_DIS * TOP_COS, STEADY_DIS * TOP_SIN, 180.0 + height_offset]

### ================

# DROP POSITION
DROP_DIS = L1 - 25.0
N1_DROP = [DROP_DIS * BOX_COS - 40.0 * BOX_COS, DROP_DIS * BOX_SIN + 40.0 * BOX_SIN, 70.0]
N2_DROP = [DROP_DIS * BOX_COS, DROP_DIS * BOX_SIN, 70.0]
N3_DROP = [DROP_DIS * BOX_COS + 40.0 * BOX_COS, DROP_DIS * BOX_SIN - 40.0 * BOX_SIN, 70.0]


# GIFT POSITION

FIRST_GIFT_Z = 62.0
SECOND_GIFT_Z = 87.0
THIRD_GIFT_Z = 112.0

# 1층
GIFT_4_DIS = L1 + 75.5
GIFT_4 = [GIFT_4_DIS * BOX_COS, GIFT_4_DIS * BOX_SIN, FIRST_GIFT_Z]

GIFT_5_DIS = L1 + 45.5
GIFT_5 = [GIFT_5_DIS * BOX_COS, GIFT_5_DIS * BOX_SIN, FIRST_GIFT_Z]

GIFT_6_DIS = L1 + 15.5
GIFT_6 = [GIFT_6_DIS * BOX_COS, GIFT_6_DIS * BOX_SIN, FIRST_GIFT_Z]

# 2층
GIFT_7_DIS = L1 + 60.5
GIFT_7 = [GIFT_7_DIS * BOX_COS, GIFT_7_DIS * BOX_SIN, SECOND_GIFT_Z]

GIFT_8_DIS = L1 + 30.5
GIFT_8 = [GIFT_8_DIS * BOX_COS, GIFT_8_DIS * BOX_SIN, SECOND_GIFT_Z]

# 3층
GIFT_9_DIS = L1 + 45.5
GIFT_9 = [GIFT_9_DIS * BOX_COS, GIFT_9_DIS * BOX_SIN, THIRD_GIFT_Z]

FAST_MOVE_Z_OFFSET = 30.0

# ================

coordinate_list = [
    # 서랍 열기
    [HANDLE * BOX_COS, HANDLE * BOX_SIN, 70.0, False, 'fast_move'],
    [HANDLE * BOX_COS, HANDLE * BOX_SIN, 30.0, False, 'detailed_move'], # 서랍 위치까지 하강
    [HANDLE * BOX_COS, HANDLE * BOX_SIN, 30.0, True, 'pick'], # 서랍 집기
    [L1_OPEN * BOX_COS, L1_OPEN * BOX_SIN, 30.0, True, 'detailed_move'], # 서랍 위치에서 서랍 열기 위치로 이동
    [L1_OPEN * BOX_COS, L1_OPEN * BOX_SIN, 30.0, False, 'place'], # 서랍 놓기
    [L1_OPEN * BOX_COS, L1_OPEN * BOX_SIN, 190.0, False, 'detailed_move'],
    # 초기 자세로 이동
    [180.0, 0.0, 190.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 1번 큐브 PICK =======================
    [TOP_1[0], TOP_1[1], TOP_1[2] + PICK_OFFSET, False, 'fast_move'],  # 안정 위치로 이동
    [TOP_1[0], TOP_1[1], TOP_1[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_1[0], TOP_1[1], TOP_1[2], True, 'pick'],  # 박스 집기
    [TOP_1[0], TOP_1[1], TOP_1[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 1번 큐브 PLACE
    [N1_DROP[0], N1_DROP[1], STEADY[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [N1_DROP[0], N1_DROP[1], N1_DROP[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [N1_DROP[0], N1_DROP[1], N1_DROP[2], False, 'place'],  # 1번 큐브 위치로 이동
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 2번 큐브 PICK =======================
    [TOP_2[0], TOP_2[1], TOP_2[2] + PICK_OFFSET, False, 'fast_move'],  # 안정 위치로 이동
    [TOP_2[0], TOP_2[1], TOP_2[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_2[0], TOP_2[1], TOP_2[2], True, 'pick'],  # 박스 집기
    [TOP_2[0], TOP_2[1], TOP_2[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 2번 큐브 PLACE
    [N2_DROP[0], N2_DROP[1], STEADY[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [N2_DROP[0], N2_DROP[1], N2_DROP[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [N2_DROP[0], N2_DROP[1], N2_DROP[2], False, 'place'],  # 1번 큐브 위치로 이동
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 3번 큐브 PICK =======================
    [TOP_3[0], TOP_3[1], TOP_3[2] + PICK_OFFSET, False, 'fast_move'],  # 안정 위치로 이동
    [TOP_3[0], TOP_3[1], TOP_3[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_3[0], TOP_3[1], TOP_3[2], True, 'pick'],  # 박스 집기
    [TOP_3[0], TOP_3[1], TOP_3[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 3번 큐브 PLACE
    [N3_DROP[0], N3_DROP[1], STEADY[2], True, 'fast_move'],
    [N3_DROP[0], N3_DROP[1], N3_DROP[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [N3_DROP[0], N3_DROP[1], N3_DROP[2], False, 'place'],  # 1번 큐브 위치로 이동
    
    # 상자를 닫기 위한 위치로 이동
    [N3_DROP[0], N3_DROP[1], N3_DROP[2] + PICK_OFFSET, False, 'fast_move'],
    [L1_OPEN * BOX_COS, L1_OPEN * BOX_SIN, N3_DROP[2] + PICK_OFFSET, False, 'fast_move'],
    [L1_OPEN * BOX_COS, L1_OPEN * BOX_SIN, 30.0, False, 'detailed_move'], # 서랍 놓기
    [HANDLE * BOX_COS, HANDLE * BOX_SIN, 30.0, False, 'detailed_move'],
    [HANDLE * BOX_COS, HANDLE * BOX_SIN, 70.0, False, 'detailed_move'],
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 4번 큐브 PICK =======================
    [STEADY[0], STEADY[1], STEADY[2], False, 'fast_move'],  # 안정 위치로 이동
    [TOP_4[0], TOP_4[1], TOP_4[2] + PICK_OFFSET, False, 'fast_move'],  # 박스 위치로 이동
    [TOP_4[0], TOP_4[1], TOP_4[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_4[0], TOP_4[1], TOP_4[2], True, 'pick'],  # 박스 집기
    [TOP_4[0], TOP_4[1], TOP_4[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 4번 큐브 PLACE
    [GIFT_4[0], GIFT_4[1], STEADY[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_4[0], GIFT_4[1], GIFT_4[2] + FAST_MOVE_Z_OFFSET, True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_4[0], GIFT_4[1], GIFT_4[2], True, 'detailed_move'],  # 1번 큐브 위치로 이동
    [GIFT_4[0], GIFT_4[1], GIFT_4[2], False, 'place'],  # 1번 큐브 위치로 이동
    [GIFT_4[0], GIFT_4[1], GIFT_4[2] + FAST_MOVE_Z_OFFSET, True, 'detailed_move'],  # 1번 큐브 위치로 이동
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 5번 큐브 PICK =======================
    [STEADY[0], STEADY[1], STEADY[2], False, 'fast_move'],  # 안정 위치로 이동
    [TOP_5[0], TOP_5[1], TOP_5[2] + PICK_OFFSET, False, 'fast_move'],  # 박스 위치로 이동
    [TOP_5[0], TOP_5[1], TOP_5[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_5[0], TOP_5[1], TOP_5[2], True, 'pick'],  # 박스 집기
    [TOP_5[0], TOP_5[1], TOP_5[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 5번 큐브 PLACE
    [GIFT_5[0], GIFT_5[1], STEADY[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_5[0], GIFT_5[1], GIFT_5[2] + FAST_MOVE_Z_OFFSET, True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_5[0], GIFT_5[1], GIFT_5[2], True, 'detailed_move'],  # 1번 큐브 위치로 이동
    [GIFT_5[0], GIFT_5[1], GIFT_5[2], False, 'place'],  # 1번 큐브 위치로 이동
    [GIFT_5[0], GIFT_5[1], GIFT_5[2] + FAST_MOVE_Z_OFFSET, True, 'detailed_move'],  # 1번 큐브 위치로 이동
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 6번 큐브 PICK =======================
    [STEADY[0], STEADY[1], STEADY[2], False, 'fast_move'],  # 안정 위치로 이동
    [TOP_6[0], TOP_6[1], TOP_6[2] + PICK_OFFSET, False, 'fast_move'],  # 박스 위치로 이동
    [TOP_6[0], TOP_6[1], TOP_6[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_6[0], TOP_6[1], TOP_6[2], True, 'pick'],  # 박스 집기
    [TOP_6[0], TOP_6[1], TOP_6[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 6번 큐브 PLACE
    [GIFT_6[0], GIFT_6[1], STEADY[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_6[0], GIFT_6[1], GIFT_6[2] + FAST_MOVE_Z_OFFSET, True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_6[0], GIFT_6[1], GIFT_6[2], True, 'detailed_move'],  # 1번 큐브 위치로 이동
    [GIFT_6[0], GIFT_6[1], GIFT_6[2], False, 'place'],  # 1번 큐브 위치로 이동
    [GIFT_6[0], GIFT_6[1], GIFT_6[2] + FAST_MOVE_Z_OFFSET, True, 'detailed_move'],  # 1번 큐브 위치로 이동
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 7번 큐브 PICK =======================
    [STEADY[0], STEADY[1], STEADY[2], False, 'fast_move'],  # 안정 위치로 이동
    [TOP_7[0], TOP_7[1], TOP_7[2] + PICK_OFFSET, False, 'fast_move'],  # 박스 위치로 이동
    [TOP_7[0], TOP_7[1], TOP_7[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_7[0], TOP_7[1], TOP_7[2], True, 'pick'],  # 박스 집기
    [TOP_7[0], TOP_7[1], TOP_7[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 7번 큐브 PLACE
    [GIFT_7[0], GIFT_7[1], STEADY[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_7[0], GIFT_7[1], GIFT_7[2] + FAST_MOVE_Z_OFFSET, True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_7[0], GIFT_7[1], GIFT_7[2], True, 'detailed_move'],  # 1번 큐브 위치로 이동
    [GIFT_7[0], GIFT_7[1], GIFT_7[2], False, 'place'],  # 1번 큐브 위치로 이동
    [GIFT_7[0], GIFT_7[1], GIFT_7[2] + FAST_MOVE_Z_OFFSET, True, 'detailed_move'],  # 1번 큐브 위치로 이동
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 8번 큐브 PICK =======================
    [STEADY[0], STEADY[1], STEADY[2], False, 'fast_move'],  # 안정 위치로 이동
    [TOP_8[0], TOP_8[1], TOP_8[2] + PICK_OFFSET, False, 'fast_move'],  # 박스 위치로 이동
    [TOP_8[0], TOP_8[1], TOP_8[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_8[0], TOP_8[1], TOP_8[2], True, 'pick'],  # 박스 집기
    [TOP_8[0], TOP_8[1], TOP_8[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 8번 큐브 PLACE
    [GIFT_8[0], GIFT_8[1], STEADY[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_8[0], GIFT_8[1], GIFT_8[2] + FAST_MOVE_Z_OFFSET, True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_8[0], GIFT_8[1], GIFT_8[2], True, 'detailed_move'],  # 1번 큐브 위치로 이동
    [GIFT_8[0], GIFT_8[1], GIFT_8[2], False, 'place'],  # 1번 큐브 위치로 이동
    [GIFT_8[0], GIFT_8[1], GIFT_8[2] + FAST_MOVE_Z_OFFSET, True, 'detailed_move'],  # 1번 큐브 위치로 이동
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 9번 큐브 PICK =======================
    [STEADY[0], STEADY[1], STEADY[2], False, 'fast_move'],  # 안정 위치로 이동
    [TOP_9[0], TOP_9[1], TOP_9[2] + PICK_OFFSET, False, 'fast_move'],  # 박스 위치로 이동
    [TOP_9[0], TOP_9[1], TOP_9[2], False, 'detailed_move'],  # 박스 위치로 이동
    [TOP_9[0], TOP_9[1], TOP_9[2], True, 'pick'],  # 박스 집기
    [TOP_9[0], TOP_9[1], TOP_9[2] + PICK_OFFSET, True, 'detailed_move'],  # 박스 위치로 이동
    [STEADY[0], STEADY[1], STEADY[2], True, 'fast_move'],  # 안정 위치로 이동
    
    # 9번 큐브 PLACE
    [GIFT_9[0], GIFT_9[1], STEADY[2], True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_9[0], GIFT_9[1], GIFT_9[2] + FAST_MOVE_Z_OFFSET, True, 'fast_move'],  # 1번 큐브 위치로 이동
    [GIFT_9[0], GIFT_9[1], GIFT_9[2], True, 'detailed_move'],  # 1번 큐브 위치로 이동
    [GIFT_9[0], GIFT_9[1], GIFT_9[2], False, 'place'],  # 1번 큐브 위치로 이동
    [GIFT_9[0], GIFT_9[1], GIFT_9[2] + FAST_MOVE_Z_OFFSET, True, 'detailed_move'],  # 1번 큐브 위치로 이동
    
    # 초기 자세로 이동
    [150.0, 0.0, 250.0, False, 'fast_move'],  # 초기 위치로 이동
    
    # 초기 자세로 이동
    [150.0, -100.0, 200.0, False, 'fast_move'],
    [150.0, 100.0, 150.0, False, 'fast_move'],
    [150.0, -100.0, 200.0, False, 'fast_move'],
    [150.0, 100.0, 150.0, False, 'fast_move'],
    [150.0, -100.0, 200.0, False, 'fast_move'],
    [150.0, 100.0, 150.0, False, 'fast_move'],
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



