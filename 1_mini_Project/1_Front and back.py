"""
이번 코드는 기존의 가속/감속 방식 대신 숫자 키패드(1~0)를 활용한 기어 변속 방식의 로봇 주행 제어 스크립트입니다. 
숫자 1번(-200)부터 0번(200)까지 각 키에 특정 속도값을 미리 매핑하여,
사용자가 숫자를 누르는 즉시 해당 속도로 로봇이 구동되도록 설계되었습니다.
기술적으로는 다이나믹셀의 4Byte 속도 제어 주소에 직접 접근하며, 
왼쪽과 오른쪽 모터의 회전 방향을 반전시켜 직진 주행의 편의성을 확보한 것이 특징입니다.
이를 통해 복잡한 조작 없이도 정해진 속도 단계에 맞춰 로봇의 성능을 신속하고 정확하게 테스트할 수 있습니다.
"""

import os
import sys
import time
import select

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch    

from dynamixel_sdk import *

# 다이나믹셀 기본 설정
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'
DXL_IDS = [1, 2, 3, 4]  # 다이나믹셀 ID 리스트

# 다이나믹셀 주소값
ADDR_TORQUE_ENABLE = 64        
ADDR_OPERATING_MODE = 11       
ADDR_GOAL_VELOCITY = 104       
ADDR_PRESENT_VELOCITY = 128    

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_MODE = 1 

# 포트 및 패킷 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# 포트 열기
if not portHandler.openPort():
    print("Failed to open the port")
    sys.exit(1)

# 보드레이트 설정
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    sys.exit(1)

# 다이나믹셀 속도 모드 설정
for dxl_id in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)  
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)  
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)   

try:
    while True:
        key = getch()

        # 백스페이스 입력 시 종료
        if key in ('\x08', '\x7f'):  # '\x08' (Windows), '\x7f' (Linux/macOS)
            print("\n프로그램을 종료합니다.")
            break

        # 각 키에 맞는 속도 값 설정
        velocity_map = {
            '1': -200, '2': -150, '3': -100, '4': -50,
            '5': 0, '6': 0, '7': 50, '8': 100, '9': 150, '0': 200
        }
        if key in velocity_map:
            goal_velocity = velocity_map[key]

            # 3번, 4번 모터 속도 설정
            packetHandler.write4ByteTxRx(portHandler, DXL_IDS[0], ADDR_GOAL_VELOCITY, goal_velocity)
            packetHandler.write4ByteTxRx(portHandler, DXL_IDS[1], ADDR_GOAL_VELOCITY, -goal_velocity)
            packetHandler.write4ByteTxRx(portHandler, DXL_IDS[2], ADDR_GOAL_VELOCITY, goal_velocity)
            packetHandler.write4ByteTxRx(portHandler, DXL_IDS[3], ADDR_GOAL_VELOCITY, -goal_velocity)
            print(f"속도 : {goal_velocity}")

except KeyboardInterrupt:
    print("\nCtrl+C 입력 감지! 프로그램을 종료합니다.")

# 모터 정지 후 종료
for dxl_id in DXL_IDS:
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, 0)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()
print("프로그램 종료 완료!")
