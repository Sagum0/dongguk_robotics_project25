#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk import *  # PortHandler, PacketHandler 등 제공

import threading

##############################################
# 글로벌 상수 정의
##############################################

# AX-18A (Protocol 1.0) 설정
AX_BAUDRATE = 115200
AX_ID_LIST = [1, 2, 3, 4]
AX_PROTOCOL_VERSION = 1.0
AX_DEVICENAME = '/dev/ttyUSB0'

# AX-18A 제어 테이블 주소 (AX-12A와 유사한 제어 테이블 사용 가정)
AX_OPERATING_MODE_ADDR   = 11   # (참고: AX 시리즈는 기본적으로 Position 모드이므로 지원하지 않을 수 있음)
AX_GOAL_POSITION_ADDR    = 30
AX_PRESENT_POSITION_ADDR = 36
AX_MOVING_SPEED_ADDR     = 32
AX_TORQUE_ENABLE_ADDR    = 24

# XC-330 (Protocol 2.0) 설정
XC_BAUDRATE = 115200
XC_ID = 5
XC_PROTOCOL_VERSION = 2.0
XC_DEVICENAME = '/dev/ttyUSB1'

# XC-330 제어 테이블 주소 (XM430과 유사한 제어 테이블 사용 가정)
XC_OPERATING_MODE_ADDR   = 11
XC_GOAL_POSITION_ADDR    = 116
XC_PRESENT_POSITION_ADDR = 132
XC_PRESENT_SPEED_ADDR    = 128
XC_TORQUE_ENABLE_ADDR    = 64

# Position 모드 값 (두 모터 모두 Position 제어를 위해 3으로 설정)
POSITION_MODE = 3

##############################################
# MotorReady Node 클래스 정의
##############################################

class MotorReady(Node):
    def __init__(self):
        super().__init__('motor_ready')
        self.get_logger().info("MotorReady 노드 시작")

        # Publisher: present position을 100ms마다 publish
        self.present_pos_pub = self.create_publisher(Float32MultiArray, '/robotics_present_postion', 10)
        # Subscriber: goal position 명령을 수신
        self.goal_pos_sub = self.create_subscription(Float32MultiArray, '/robotics_goal_position', self.goal_pos_callback, 10)

        # 멀티스레드 환경에서 포트 접근을 위한 Lock 생성
        self.lock = threading.Lock()

        ##############################################
        # 모터 통신 초기화: AX 모터와 XC 모터를 각각 초기화
        ##############################################

        # AX 모터 (Protocol 1.0)
        self.ax_port = PortHandler(AX_DEVICENAME)
        self.ax_packet = PacketHandler(AX_PROTOCOL_VERSION)
        if not self.ax_port.openPort():
            self.get_logger().error("AX 모터 포트 열기 실패: {}".format(AX_DEVICENAME))
        if not self.ax_port.setBaudRate(AX_BAUDRATE):
            self.get_logger().error("AX 모터 baudrate 설정 실패")
        
        # XC 모터 (Protocol 2.0)
        self.xc_port = PortHandler(XC_DEVICENAME)
        self.xc_packet = PacketHandler(XC_PROTOCOL_VERSION)
        if not self.xc_port.openPort():
            self.get_logger().error("XC 모터 포트 열기 실패: {}".format(XC_DEVICENAME))
        if not self.xc_port.setBaudRate(XC_BAUDRATE):
            self.get_logger().error("XC 모터 baudrate 설정 실패")

        ##############################################
        # 각 모터별 초기 설정 (동작 모드 설정 및 Torque Enable)
        ##############################################

        # AX 모터 초기화 (여러 개)
        for motor_id in AX_ID_LIST:
            # 동작 모드 설정 (AX-18A는 기본적으로 Position 제어이므로 실제 적용 안될 수 있음)
            dxl_comm_result, dxl_error = self.ax_packet.write1ByteTxRx(self.ax_port, motor_id, AX_OPERATING_MODE_ADDR, POSITION_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn("AX Motor {}: 동작 모드 설정 실패({})".format(motor_id, self.ax_packet.getTxRxResult(dxl_comm_result)))
            # Torque Enable (1: enable)
            dxl_comm_result, dxl_error = self.ax_packet.write1ByteTxRx(self.ax_port, motor_id, AX_TORQUE_ENABLE_ADDR, 1)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn("AX Motor {}: Torque 활성화 실패({})".format(motor_id, self.ax_packet.getTxRxResult(dxl_comm_result)))

        # XC 모터 초기화 (단일 모터)
        dxl_comm_result, dxl_error = self.xc_packet.write1ByteTxRx(self.xc_port, XC_ID, XC_OPERATING_MODE_ADDR, POSITION_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn("XC Motor: 동작 모드 설정 실패({})".format(self.xc_packet.getTxRxResult(dxl_comm_result)))
        dxl_comm_result, dxl_error = self.xc_packet.write1ByteTxRx(self.xc_port, XC_ID, XC_TORQUE_ENABLE_ADDR, 1)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn("XC Motor: Torque 활성화 실패({})".format(self.xc_packet.getTxRxResult(dxl_comm_result)))

        # 100ms 주기 타이머: present position publish
        self.timer = self.create_timer(0.1, self.publish_present_position)

    ##############################################
    # 구독 콜백: goal position 수신 시 각 모터에 명령 전송
    ##############################################
    def goal_pos_callback(self, msg):
        # msg.data: [AX1, AX2, AX3, AX4, XC] 순서로 goal position 값이 들어있다고 가정
        if len(msg.data) < 5:
            self.get_logger().error("goal position 배열 길이가 5 미만입니다.")
            return

        with self.lock:
            # AX 모터 (Protocol 1.0) 처리: write2ByteTxRx 사용
            for i, motor_id in enumerate(AX_ID_LIST):
                goal_pos = int(msg.data[i])
                dxl_comm_result, dxl_error = self.ax_packet.write2ByteTxRx(self.ax_port, motor_id, AX_GOAL_POSITION_ADDR, goal_pos)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn("AX Motor {}: Goal Position 쓰기 실패({})".format(motor_id, self.ax_packet.getTxRxResult(dxl_comm_result)))
            # XC 모터 (Protocol 2.0) 처리: write4ByteTxRx 사용 (4바이트 쓰기)
            goal_pos = int(msg.data[4])
            dxl_comm_result, dxl_error = self.xc_packet.write4ByteTxRx(self.xc_port, XC_ID, XC_GOAL_POSITION_ADDR, goal_pos)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn("XC Motor: Goal Position 쓰기 실패({})".format(self.xc_packet.getTxRxResult(dxl_comm_result)))

    ##############################################
    # 타이머 콜백: 모든 모터의 present position 읽고 publish
    ##############################################
    def publish_present_position(self):
        msg = Float32MultiArray()
        positions = []
        with self.lock:
            # AX 모터: read2ByteTxRx 사용하여 present position 읽기
            for motor_id in AX_ID_LIST:
                pos, dxl_comm_result, dxl_error = self.ax_packet.read2ByteTxRx(self.ax_port, motor_id, AX_PRESENT_POSITION_ADDR)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn("AX Motor {}: Present Position 읽기 실패({})".format(motor_id, self.ax_packet.getTxRxResult(dxl_comm_result)))
                    pos = 0
                positions.append(float(pos))
            # XC 모터: read4ByteTxRx 사용 (4바이트 읽기)
            pos, dxl_comm_result, dxl_error = self.xc_packet.read4ByteTxRx(self.xc_port, XC_ID, XC_PRESENT_POSITION_ADDR)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn("XC Motor: Present Position 읽기 실패({})".format(self.xc_packet.getTxRxResult(dxl_comm_result)))
                pos = 0
            positions.append(float(pos))
        msg.data = positions
        self.present_pos_pub.publish(msg)

##############################################
# 메인 함수: MultiThreadedExecutor 사용
##############################################
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
