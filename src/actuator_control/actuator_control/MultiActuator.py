import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import PythonLibMightyZap_PC as MightyZap
import asyncio

# 상수 정의
SERIAL_PORT = '/dev/ttyUSB0'    # 시리얼 포트
BAUD_RATE = 57600               # 전송 속도
ACTUATOR_COUNT = 2              # 액츄에이터 개수
ACTUATOR_IDS = [3, 0]           # 액츄에이터 ID 목록
OPEN_POSITIONS = [3500, 4000]   # 개방 위치
ERROR_BOUNDARY = 10             # 위치 오차 허용 범위

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('actuator_controller')
        self.status = None  # True : 액츄에이터가 늘어난 상태, False : 액추에이터가 줄어든 상태, None : 초기 값
        self.cam_cover_subscriber = self.create_subscription(Bool, 'rcs/cam_cover', self.cam_cover_cb, 10)  # Subscriber 등록

        # MightyZap 초기화
        self.initialize_mighty_zap()

    def initialize_mighty_zap(self):
        """MightyZap 초기화 및 액츄에이터 비활성화"""
        MightyZap.OpenMightyZap(SERIAL_PORT, BAUD_RATE)
        for actuator_id in ACTUATOR_IDS:
            MightyZap.ForceEnable(actuator_id, 0)
        print("==========================================================")
        print("SYSYEM START")

    def cam_cover_cb(self, msg):
        """
            카메라 커버 명령 콜백 함수
            Args:
                msg.data:
                    True    :   Cover 닫힘 (Actuator Position = OPEN_POSITION)
                    False   :   Cover 열림 (Actuator Position = 0)
        """
        print("==========================================================")
        print(f"Received : {msg.data}")
        print("==========================================================")
        target_positions = self.calculate_target_positions(msg.data)    # msg에 따른 target_position 명령 지정

        if self.status != msg.data:  # 상태가 변경된 경우
            asyncio.run(self.move_actuators(target_positions))  # 비동기로 액츄에이터 이동완료 될 때 까지 대기
            self.status = msg.data  # 상태 업데이트

    def calculate_target_positions(self, flag):
        """
            열림 상태에 따라 목표 위치 계산
            Args:
                flag:
                    True    :   target_position = OPEN_POSITION
                    False   :   target_position = 0
        """
        return [pos * int(flag) for pos in OPEN_POSITIONS]

    async def move_actuators(self, target_positions):
        """
            액츄에이터를 목표 위치로 비동기 이동
            Args:
                target_positions    :   액츄에이터의 목표 위치 리스트
        """
        tasks = []
        for actuator_id, target_position in zip(ACTUATOR_IDS, target_positions):
            self.get_logger().info(f"position command : {target_position}")
            MightyZap.GoalPosition(actuator_id, target_position)  # 액츄에이터에 target_position 명령을 전달
            tasks.append(self.wait_for_position(actuator_id, target_position))  # 지정된 위치에 도달할 때까지 대기하는 작업 추가
        await asyncio.gather(*tasks)  # 모든 작업 비동기적으로 실행

    async def wait_for_position(self, actuator_id, target_position):
        """
            지정한 위치에 도달할 때까지 비동기 대기
            Args:
                actuator_id : 도착 대기 하는 액츄에이터의 id
                target_position : 액츄에이터의 목표 위치
        """
        # ERROR_BOUNDARY 안에 들어올 때 까지 대기
        while abs(MightyZap.PresentPosition(actuator_id) - target_position) > ERROR_BOUNDARY:
            await asyncio.sleep(0.1)
            
        MightyZap.ForceEnable(actuator_id, 0)   # 액츄에이터 Force 비활성화
        self.get_logger().info(f"Actuator {actuator_id} reached position: {MightyZap.PresentPosition(actuator_id)}")

def main(args=None):
    rclpy.init(args=args)
    commandSubscriber = CommandSubscriber()
    
    try:
        rclpy.spin(commandSubscriber)  # 노드 실행
    except Exception as e:
        print(f"Exception occurred: {e}")  # 에러 로그
    finally:
        for actuator_id in ACTUATOR_IDS:
            MightyZap.ForceEnable(actuator_id, 0)  # 액츄에이터 비활성화
        MightyZap.CloseMightyZap()  # MightyZap 종료
        commandSubscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  # 메인 함수 호출
