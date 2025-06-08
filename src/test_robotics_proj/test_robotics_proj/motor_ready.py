#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from threading import Lock

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
)

class MotorReadyNode(Node):
    def __init__(self):
        super().__init__('motor_ready')
        self.lock = Lock()

        # Dynamixel 설정
        self.DEVICENAME_1_4         = '/dev/ttyUSB0'
        self.DEVICENAME_5           = '/dev/ttyUSB1'
        self.BAUDRATE               = 57600
        self.PROTOCOL_VERSION       = 2.0

        # 모터 ID (모두 Joint로 사용)
        self.JOINT_IDS_1_4          = [1, 2, 3, 4]
        self.JOINT_ID_5             = 5

        # Control table 주소
        self.ADDR_TORQUE_ENABLE     = 64
        self.ADDR_OPERATING_MODE    = 11
        self.ADDR_PROFILE_VELOCITY  = 112
        self.ADDR_GOAL_POSITION     = 116
        self.ADDR_PRESENT_POSITION  = 132

        # 핸들러 생성
        self.portHandler_1_4   = PortHandler(self.DEVICENAME_1_4)
        self.packetHandler_1_4 = PacketHandler(self.PROTOCOL_VERSION)
        self.portHandler_5     = PortHandler(self.DEVICENAME_5)
        self.packetHandler_5   = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler_1_4.openPort():
            self.get_logger().error(f"포트를 열 수 없습니다: {self.DEVICENAME_1_4}")
            return
        if not self.portHandler_1_4.setBaudRate(self.BAUDRATE):
            self.get_logger().error(f"보드레이트 설정 실패: {self.BAUDRATE}")
            return

        if not self.portHandler_5.openPort():
            self.get_logger().error(f"포트를 열 수 없습니다: {self.DEVICENAME_5}")
            return
        if not self.portHandler_5.setBaudRate(self.BAUDRATE):
            self.get_logger().error(f"보드레이트 설정 실패: {self.BAUDRATE}")
            return

        # 초기 설정: 1~4번
        for motor_id in self.JOINT_IDS_1_4:
            self._motor_init(self.portHandler_1_4, self.packetHandler_1_4, motor_id)
        # 5번
        self._motor_init(self.portHandler_5, self.packetHandler_5, self.JOINT_ID_5)
                
        self.get_logger().info(' Motor Connected! ')

        # 구독자
        self.sub_target_pulse = self.create_subscription(
            Float32MultiArray,
            '/robotics/abs/target_pulse',
            self.callback_target_pulse,
            10
        )
        self.sub_speed = self.create_subscription(
            Float32MultiArray,
            '/robotics/abs/speed',
            self.callback_speed,
            10
        )

        # 발행자 (60Hz)
        self.pub_present_pulse = self.create_publisher(
            Float32MultiArray,
            '/robotics/abs/pulse',
            10
        )
        timer_period = 1.0 / 60.0
        self.timer = self.create_timer(timer_period, self.timer_read_and_publish)

    def _motor_init(self, portHandler, packetHandler, motor_id):
        # 1) 토크 비활성화 (0)
        _, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, motor_id,
            self.ADDR_TORQUE_ENABLE, 0
        )
        if dxl_error != 0:
            self.get_logger().warn(f"ID {motor_id}: 토크 비활성화 오류 ({dxl_error})")

        # 2) Position Control 모드(3) 설정
        _, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, motor_id,
            self.ADDR_OPERATING_MODE, 3
        )
        if dxl_error != 0:
            self.get_logger().warn(f"ID {motor_id}: 운영 모드 설정 오류 ({dxl_error})")

        # 3) 토크 활성화 (1)
        _, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, motor_id,
            self.ADDR_TORQUE_ENABLE, 1
        )
        if dxl_error != 0:
            self.get_logger().warn(f"ID {motor_id}: 토크 활성화 오류 ({dxl_error})")

        # 4) 초기 프로파일 속도 0
        _, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, motor_id,
            self.ADDR_PROFILE_VELOCITY, 0
        )
        if dxl_error != 0:
            self.get_logger().warn(f"ID {motor_id}: 초기 속도 설정 오류 ({dxl_error})")

    def callback_target_pulse(self, msg: Float32MultiArray):
        with self.lock:
            data = msg.data
            if len(data) < 5:
                self.get_logger().warn("target_pulse 데이터 길이 부족")
                return
            # 1~4번
            for idx, motor_id in enumerate(self.JOINT_IDS_1_4):
                goal_pulse = int(data[idx])
                _, dxl_error = self.packetHandler_1_4.write4ByteTxRx(
                    self.portHandler_1_4, motor_id,
                    self.ADDR_GOAL_POSITION, goal_pulse
                )
                if dxl_error != 0:
                    self.get_logger().warn(f"ID {motor_id}: 목표 위치 쓰기 오류 ({dxl_error})")
            # 5번
            goal_pulse_5 = int(data[4])
            _, dxl_error = self.packetHandler_5.write4ByteTxRx(
                self.portHandler_5, self.JOINT_ID_5,
                self.ADDR_GOAL_POSITION, goal_pulse_5
            )
            if dxl_error != 0:
                self.get_logger().warn(f"ID {self.JOINT_ID_5}: 목표 위치 쓰기 오류 ({dxl_error})")

    def callback_speed(self, msg: Float32MultiArray):
        with self.lock:
            data = msg.data
            if len(data) < 5:
                self.get_logger().warn("speed 데이터 길이 부족")
                return
            # 1~4번
            for idx, motor_id in enumerate(self.JOINT_IDS_1_4):
                vel = int(data[idx])
                _, dxl_error = self.packetHandler_1_4.write4ByteTxRx(
                    self.portHandler_1_4, motor_id,
                    self.ADDR_PROFILE_VELOCITY, vel
                )
                if dxl_error != 0:
                    self.get_logger().warn(f"ID {motor_id}: 프로파일 속도 쓰기 오류 ({dxl_error})")
            # 5번
            vel_5 = int(data[4])
            _, dxl_error = self.packetHandler_5.write4ByteTxRx(
                self.portHandler_5, self.JOINT_ID_5,
                self.ADDR_PROFILE_VELOCITY, vel_5
            )
            if dxl_error != 0:
                self.get_logger().warn(f"ID {self.JOINT_ID_5}: 프로파일 속도 쓰기 오류 ({dxl_error})")

    def timer_read_and_publish(self):
        with self.lock:
            positions = []
            # 1~4번
            for motor_id in self.JOINT_IDS_1_4:
                present_pos, _, dxl_error = self.packetHandler_1_4.read4ByteTxRx(
                    self.portHandler_1_4, motor_id,
                    self.ADDR_PRESENT_POSITION
                )
                if dxl_error != 0:
                    self.get_logger().warn(f"ID {motor_id}: 현재 위치 읽기 오류 ({dxl_error})")
                    positions.append(0.0)
                else:
                    positions.append(float(present_pos))
            # 5번
            present_pos_5, _, dxl_error = self.packetHandler_5.read4ByteTxRx(
                self.portHandler_5, self.JOINT_ID_5,
                self.ADDR_PRESENT_POSITION
            )
            if dxl_error != 0:
                self.get_logger().warn(f"ID {self.JOINT_ID_5}: 현재 위치 읽기 오류 ({dxl_error})")
                positions.append(0.0)
            else:
                positions.append(float(present_pos_5))

            msg = Float32MultiArray()
            msg.data = positions
            self.pub_present_pulse.publish(msg)

    def destroy_node(self):
        self.portHandler_1_4.closePort()
        self.portHandler_5.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorReadyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
