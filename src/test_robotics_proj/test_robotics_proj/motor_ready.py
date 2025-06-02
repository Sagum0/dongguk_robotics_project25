#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk import *  # PortHandler, PacketHandler 등 제공

import threading
import numpy as np

##############################################
# 글로벌 상수 정의
##############################################
AX_BAUDRATE = 115200
AX_ID_LIST = [1, 2, 3, 4]
AX_PROTOCOL_VERSION = 1.0
AX_DEVICENAME = '/dev/ttyUSB0'

AX_GOAL_POSITION_ADDR    = 30
AX_PRESENT_POSITION_ADDR = 36
AX_MOVING_SPEED_ADDR     = 32
AX_TORQUE_ENABLE_ADDR    = 24

XC_BAUDRATE = 115200
XC_ID = 5
XC_PROTOCOL_VERSION = 2.0
XC_DEVICENAME = '/dev/ttyUSB0'

XC_OPERATING_MODE_ADDR   = 11
XC_GOAL_POSITION_ADDR    = 116
XC_PRESENT_POSITION_ADDR = 132
XC_PRESENT_SPEED_ADDR    = 128
XC_TORQUE_ENABLE_ADDR    = 64

POSITION_MODE = 3

##############################################
# 과제 번호 설정 (변경 불필요)
##############################################
TASK_NUMBER = 2  # 중간 과제: 1번 / 기말 과제: 2번

##############################################
# MotorReady Node 클래스 정의
##############################################
class MotorReady(Node):
    def __init__(self):
        super().__init__('motor_ready')
        self.get_logger().info("MotorReady 노드 시작")

        # 1) 현재 위치(present position) 퍼블리셔
        self.present_pos_pub = self.create_publisher(
            Float32MultiArray, '/robotics/abs/pulse', 10
        )

        # 2) 목표 위치(goal position) 구독자
        self.goal_pos_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/abs/target_pulse',
            self.goal_pos_callback,
            10
        )

        # 3) 새로운 구독자: AX 모터의 Moving Speed를 설정할 토픽
        self.speed_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/speed/pulse_speed',
            self.speed_callback,
            10
        )

        # 멀티스레드 환경에서 포트 접근을 위한 Lock 생성
        self.lock = threading.Lock()
        
        # AX 모터 초기화 (Protocol 1.0)
        self.ax_port = PortHandler(AX_DEVICENAME)
        self.ax_packet = PacketHandler(AX_PROTOCOL_VERSION)
        if not self.ax_port.openPort():
            self.get_logger().error(f"AX 모터 포트 열기 실패: {AX_DEVICENAME}")
        if not self.ax_port.setBaudRate(AX_BAUDRATE):
            self.get_logger().error("AX 모터 baudrate 설정 실패")
        
        # XC 모터 초기화 (Protocol 2.0)
        self.xc_port = PortHandler(XC_DEVICENAME)
        self.xc_packet = PacketHandler(XC_PROTOCOL_VERSION)
        if not self.xc_port.openPort():
            self.get_logger().error(f"XC 모터 포트 열기 실패: {XC_DEVICENAME}")
        if not self.xc_port.setBaudRate(XC_BAUDRATE):
            self.get_logger().error("XC 모터 baudrate 설정 실패")

        # AX 모터: Torque 활성화 및 초기 속도 제한 설정
        speed_limits = {1: 100, 2: 50, 3: 50, 4: 50}
        for motor_id in AX_ID_LIST:
            # Torque 활성화
            dxl_comm_result, _ = self.ax_packet.write1ByteTxRx(
                self.ax_port, motor_id, AX_TORQUE_ENABLE_ADDR, 1
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(
                    f"AX Motor {motor_id}: Torque 활성화 실패 ({self.ax_packet.getTxRxResult(dxl_comm_result)})"
                )
            # 초기 Moving Speed 설정
            limit_speed = speed_limits.get(motor_id, 150)
            dxl_comm_result, _ = self.ax_packet.write2ByteTxRx(
                self.ax_port, motor_id, AX_MOVING_SPEED_ADDR, limit_speed
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(
                    f"AX Motor {motor_id}: 초기 Moving Speed 설정 실패 ({self.ax_packet.getTxRxResult(dxl_comm_result)})"
                )

        # XC 모터: Position 모드 설정 및 Torque 활성화
        dxl_comm_result, _ = self.xc_packet.write1ByteTxRx(
            self.xc_port, XC_ID, XC_OPERATING_MODE_ADDR, POSITION_MODE
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn(
                f"XC Motor: 동작 모드 설정 실패 ({self.xc_packet.getTxRxResult(dxl_comm_result)})"
            )
        dxl_comm_result, _ = self.xc_packet.write1ByteTxRx(
            self.xc_port, XC_ID, XC_TORQUE_ENABLE_ADDR, 1
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn(
                f"XC Motor: Torque 활성화 실패 ({self.xc_packet.getTxRxResult(dxl_comm_result)})"
            )

        # 10ms 주기 타이머: present position publish
        self.timer = self.create_timer(0.1, self.publish_present_position)

    def goal_pos_callback(self, msg: Float32MultiArray):
        data = msg.data
        # TASK_NUMBER에 따라 데이터 길이 체크
        if TASK_NUMBER == 1:
            if len(data) < 4:
                self.get_logger().error("goal position 배열 길이가 4 미만입니다.")
                return
            theta = data  # [0]~[3]까지 AX용, 없으면 오류
        else:
            if len(data) < 5:
                self.get_logger().error("goal position 배열 길이가 5 미만입니다.")
                return
            theta = data  # [0]~[3]: AX, [4]: XC

        with self.lock:
            # AX 모터 위치(write2ByteTxRx)
            for i, motor_id in enumerate(AX_ID_LIST):
                goal_pos = int(theta[i])
                dxl_comm_result, _ = self.ax_packet.write2ByteTxRx(
                    self.ax_port, motor_id, AX_GOAL_POSITION_ADDR, goal_pos
                )
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(
                        f"AX Motor {motor_id}: Goal Position 쓰기 실패 ({self.ax_packet.getTxRxResult(dxl_comm_result)})"
                    )

            # XC 모터 위치(write4ByteTxRx)
            goal_pos_xc = int(theta[4])
            dxl_comm_result, _ = self.xc_packet.write4ByteTxRx(
                self.xc_port, XC_ID, XC_GOAL_POSITION_ADDR, goal_pos_xc
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(
                    f"XC Motor: Goal Position 쓰기 실패 ({self.xc_packet.getTxRxResult(dxl_comm_result)})"
                )

    def speed_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 4:
            self.get_logger().error("speed 배열 길이가 4 미만입니다. [1,2,3,4] 형태여야 합니다.")
            return

        with self.lock:
            # 들어온 [s1, s2, s3, s4] 각각 AX 모터의 Moving Speed 설정
            for i, motor_id in enumerate(AX_ID_LIST):
                print(f"AX Motor {motor_id} Moving Speed 설정: {data[i]}")
                speed_val = int(data[i])
                dxl_comm_result, _ = self.ax_packet.write2ByteTxRx(
                    self.ax_port, motor_id, AX_MOVING_SPEED_ADDR, speed_val
                )
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(
                        f"AX Motor {motor_id}: Moving Speed 설정 실패 ({self.ax_packet.getTxRxResult(dxl_comm_result)})"
                    )
            self.get_logger().info(f"AX 모터 속도 업데이트: {data[:4]}")

    def publish_present_position(self):
        msg = Float32MultiArray()
        positions = []
        with self.lock:
            for motor_id in AX_ID_LIST:
                pos, dxl_comm_result, _ = self.ax_packet.read2ByteTxRx(
                    self.ax_port, motor_id, AX_PRESENT_POSITION_ADDR
                )
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(
                        f"AX Motor {motor_id}: Present Position 읽기 실패 ({self.ax_packet.getTxRxResult(dxl_comm_result)})"
                    )
                    pos = 0
                positions.append(float(pos))

        msg.data = positions
        self.present_pos_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorReady()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt 감지, 노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
