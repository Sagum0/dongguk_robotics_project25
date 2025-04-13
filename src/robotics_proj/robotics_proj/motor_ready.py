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

TASK_NUMBER = 1  # 중간 과제: 1번 / 기말 과제: 2번

##############################################
# MotorReady Node 클래스 정의
##############################################
class MotorReady(Node):
    def __init__(self):
        super().__init__('motor_ready')
        self.get_logger().info("MotorReady 노드 시작")

        # Publisher: 100ms마다 present position publish
        self.present_pos_pub = self.create_publisher(Float32MultiArray, '/robotics_present_position', 10)
        # Subscriber: goal position 명령 수신
        self.goal_pos_sub = self.create_subscription(Float32MultiArray, '/robotics_goal_position', self.goal_pos_callback, 10)

        # 멀티스레드 환경에서 포트 접근을 위한 Lock 생성
        self.lock = threading.Lock()
        
        # AX 모터 (Protocol 1.0)
        self.ax_port = PortHandler(AX_DEVICENAME)
        self.ax_packet = PacketHandler(AX_PROTOCOL_VERSION)
        if not self.ax_port.openPort():
            self.get_logger().error("AX 모터 포트 열기 실패: {}".format(AX_DEVICENAME))
        if not self.ax_port.setBaudRate(AX_BAUDRATE):
            self.get_logger().error("AX 모터 baudrate 설정 실패")
        
        if TASK_NUMBER == 2:
            self.xc_port = PortHandler(XC_DEVICENAME)
            self.xc_packet = PacketHandler(XC_PROTOCOL_VERSION)
            if not self.xc_port.openPort():
                self.get_logger().error("XC 모터 포트 열기 실패: {}".format(XC_DEVICENAME))
            if not self.xc_port.setBaudRate(XC_BAUDRATE):
                self.get_logger().error("XC 모터 baudrate 설정 실패")

        # AX 모터 초기화 (동작 모드 변경은 없음)
        for motor_id in AX_ID_LIST:
            dxl_comm_result, dxl_error = self.ax_packet.write1ByteTxRx(self.ax_port, motor_id, AX_TORQUE_ENABLE_ADDR, 1)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn("AX Motor {}: Torque 활성화 실패({})".format(motor_id, self.ax_packet.getTxRxResult(dxl_comm_result)))

        if TASK_NUMBER == 2:
            dxl_comm_result, dxl_error = self.xc_packet.write1ByteTxRx(self.xc_port, XC_ID, XC_OPERATING_MODE_ADDR, POSITION_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn("XC Motor: 동작 모드 설정 실패({})".format(self.xc_packet.getTxRxResult(dxl_comm_result)))
            dxl_comm_result, dxl_error = self.xc_packet.write1ByteTxRx(self.xc_port, XC_ID, XC_TORQUE_ENABLE_ADDR, 1)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn("XC Motor: Torque 활성화 실패({})".format(self.xc_packet.getTxRxResult(dxl_comm_result)))

        # 100ms 주기 타이머: present position publish
        self.timer = self.create_timer(0.1, self.publish_present_position)

    def goal_pos_callback(self, msg):
        """
        msg.data: [AX1, AX2, AX3, AX4, (XC)] 순서의 goal position 값
        TASK_NUMBER==1이면 4개, TASK_NUMBER==2이면 5개로 구성됨.
        변환 공식:
           - AX 모터: theta = 512 + (angle/π)*1024, 제한 범위: [0,1023]
           - XC 모터: theta = 2048 + (angle/π)*4096, 제한 범위: [0,4095]
        """
        data = msg.data
        if TASK_NUMBER == 1:
            if len(data) < 4:
                self.get_logger().error("goal position 배열 길이가 4 미만입니다.")
                return
            # 리스트 컴프리헨션을 사용하여 각 theta 값 계산 및 클리핑
            theta = [int(np.clip(512 + (angle/np.pi)*1024, 0, 1023)) for angle in data[:4]]
        else:
            if len(data) < 5:
                self.get_logger().error("goal position 배열 길이가 5 미만입니다.")
                return
            theta = [int(np.clip(512 + (data[i]/np.pi)*1024, 0, 1023)) for i in range(4)]
            theta5 = int(np.clip(2048 + (data[4]/np.pi)*4096, 0, 4095))
            theta.append(theta5)

        with self.lock:
            # AX 모터 (Protocol 1.0) 처리: write2ByteTxRx 사용
            for i, motor_id in enumerate(AX_ID_LIST):
                goal_pos = theta[i]
                dxl_comm_result, dxl_error = self.ax_packet.write2ByteTxRx(self.ax_port, motor_id, AX_GOAL_POSITION_ADDR, goal_pos)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn("AX Motor {}: Goal Position 쓰기 실패({})".format(motor_id, self.ax_packet.getTxRxResult(dxl_comm_result)))
                    
            if TASK_NUMBER == 2:
                goal_pos = theta[4]
                dxl_comm_result, dxl_error = self.xc_packet.write4ByteTxRx(self.xc_port, XC_ID, XC_GOAL_POSITION_ADDR, goal_pos)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn("XC Motor: Goal Position 쓰기 실패({})".format(self.xc_packet.getTxRxResult(dxl_comm_result)))
                    
    def publish_present_position(self):
        msg = Float32MultiArray()
        positions = []
        with self.lock:
            for motor_id in AX_ID_LIST:
                pos, dxl_comm_result, dxl_error = self.ax_packet.read2ByteTxRx(self.ax_port, motor_id, AX_PRESENT_POSITION_ADDR)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn("AX Motor {}: Present Position 읽기 실패({})".format(motor_id, self.ax_packet.getTxRxResult(dxl_comm_result)))
                    pos = 0
                positions.append(float(pos))
                
            if TASK_NUMBER == 2:
                pos, dxl_comm_result, dxl_error = self.xc_packet.read4ByteTxRx(self.xc_port, XC_ID, XC_PRESENT_POSITION_ADDR)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn("XC Motor: Present Position 읽기 실패({})".format(self.xc_packet.getTxRxResult(dxl_comm_result)))
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