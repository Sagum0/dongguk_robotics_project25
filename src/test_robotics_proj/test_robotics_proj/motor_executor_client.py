#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from robotics_interfaces.srv import MotorExecutor
import numpy as np

# 로봇 링크 길이 등 (예시 그대로 둠)
L1 = 200
BOX_TH = 45

L2 = 150
TOP_TH = 30

# ================
BOX_OPEN_DISTANCE = 50.0
L1_OPEN = L1 - BOX_OPEN_DISTANCE

BOX_RAD = np.deg2rad(BOX_TH)
TOP_RAD = np.deg2rad(TOP_TH)

BOX_SIN = np.sin(BOX_RAD)
BOX_COS = np.cos(BOX_RAD)

TOP_SIN = np.sin(TOP_RAD)
TOP_COS = np.cos(TOP_RAD)

# ================

coordinate_list = [
    [L1 * BOX_SIN, L1 * BOX_COS, 150.0, False, 'move'],
    [L1 * BOX_SIN, L1 * BOX_COS,  30.0, False, 'move'],
    [L1 * BOX_SIN, L1 * BOX_COS,  30.0,  True, 'pick'],
    [L1_OPEN * BOX_SIN, L1_OPEN * BOX_COS,  30.0,  True, 'move'],
    [L1_OPEN * BOX_SIN, L1_OPEN * BOX_COS,  30.0, False, 'pick'],
    [85.0,  0.0, 270.0, False, 'move'],
    # 나머지 좌표 …
]

class MotorExecutorClient(Node):
    def __init__(self):
        super().__init__('motor_executor_client')
        self.client = self.create_client(MotorExecutor, 'motor_executor')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 서버 대기 중...')

        # 토픽 구독: 현재 좌표 받음
        self.coordinate_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/coordinate/position',
            self.coordinate_callback,
            10
        )

        # 인덱스, 목표 리스트
        self.idx = 0
        self.coordinate_list = coordinate_list

        # 마지막으로 보낸 목표(x,y,z)를 저장할 변수
        self.last_goal = None   # (x, y, z) 튜플
        # 로봇이 목표에 도달할 때까지 기다리는 플래그
        self.waiting_for_reach = False

        # 좌표 허용 오차
        self.tol = 15.0

        # 현재 실제 좌표 (처음엔 None)
        self.x = None
        self.y = None
        self.z = None

        # 첫 번째 목표 전송
        self.send_next_goal()

    def coordinate_callback(self, msg: Float32MultiArray):
        # x,y,z 값만 받는다고 가정 (msg.data = [x, y, z])
        if len(msg.data) < 3:
            self.get_logger().warn('좌표 데이터가 올바르지 않습니다.')
            return

        self.x = msg.data[0]
        self.y = msg.data[1]
        self.z = msg.data[2]
        # 디버그용 출력
        # print(f'callback 현재 위치: x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}')

        # “마지막 목표 좌표”가 설정되어 있고, 아직 목표 도달 전이라면
        if self.waiting_for_reach and self.last_goal is not None:
            gx, gy, gz = self.last_goal
            dx = abs(self.x - gx)
            dy = abs(self.y - gy)
            dz = abs(self.z - gz)
            # 모두 허용 오차(tol) 이내이면 도달했다고 간주
            if dx <= self.tol and dy <= self.tol and dz <= self.tol:
                self.get_logger().info(
                    f'목표 도달 확인({gx:.2f}, {gy:.2f}, {gz:.2f}) ± {self.tol}. '
                    f'현재({self.x:.2f}, {self.y:.2f}, {self.z:.2f})'
                )
                # 목표 도달했으므로 다음 목표 전송
                self.waiting_for_reach = False
                self.idx += 1
                self.send_next_goal()

    def send_next_goal(self):
        # 인덱스가 리스트 범위를 넘어가면 종료
        if self.idx >= len(self.coordinate_list):
            self.get_logger().info('모든 좌표 전송 완료')
            return

        x, y, z, grab, task = self.coordinate_list[self.idx]
        req = MotorExecutor.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.task = task
        req.grab = grab

        self.get_logger().info(
            f'[{self.idx}] 서비스 요청 → '
            f'x={x:.2f}, y={y:.2f}, z={z:.2f}, task={task}, grab={grab}'
        )
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

        # “서비스를 보냈다”는 의미로 last_goal과 플래그 설정
        self.last_goal = (x, y, z)
        self.waiting_for_reach = True

    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'서비스 호출 실패: {e}')
            return

        if res.success:
            self.get_logger().info(f'[{self.idx}] 서비스 동작을 시작했습니다.')
            # 실제 도달 여부는 coordinate_callback에서 체크 → 다음 호출
        else:
            self.get_logger().warn(f'[{self.idx}] 서비스 동작 실패')
            # 실패 시 재시도하거나 로직 추가 가능
            # 예를 들어 재시도: self.send_next_goal()
            # 아니면 idx += 1로 넘어가거나, 전체 종료 등

def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutorClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
