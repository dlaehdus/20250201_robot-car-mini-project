"""
다이나믹셀 모터 4개를 제어하여 키보드 입력(w, s, r)으로 로봇을 전·후진 및 정지시키는 실시간 원격 주행 제어 코드입니다.
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

# 초기 속도
goal_velocity = 0
VELOCITY_STEP = 4
VELOCITY_MAX = 200
VELOCITY_MIN = -200

try:
    while True:
        key = getch()

        # 백스페이스 입력 시 종료
        if key in ('\x08', '\x7f'):  # '\x08' (Windows), '\x7f' (Linux/macOS)
            print("\n백스페이스 입력 감지! 프로그램을 종료합니다.")
            break

        # 속도 조절
        if key == 'w':
            goal_velocity = min(goal_velocity + VELOCITY_STEP, VELOCITY_MAX)
        elif key == 's':
            goal_velocity = max(goal_velocity - VELOCITY_STEP, VELOCITY_MIN)
        elif key == 'r':
            goal_velocity = 0
        else:
            continue  # 다른 키 입력은 무시

        # 3번, 4번 모터 속도 설정
        packetHandler.write4ByteTxRx(portHandler, DXL_IDS[0], ADDR_GOAL_VELOCITY, goal_velocity)
        packetHandler.write4ByteTxRx(portHandler, DXL_IDS[1], ADDR_GOAL_VELOCITY, -goal_velocity)
        packetHandler.write4ByteTxRx(portHandler, DXL_IDS[2], ADDR_GOAL_VELOCITY, goal_velocity)
        packetHandler.write4ByteTxRx(portHandler, DXL_IDS[3], ADDR_GOAL_VELOCITY, -goal_velocity)
        print(f"현재 속도: {goal_velocity}")

except KeyboardInterrupt:
    print("\nCtrl+C 입력 감지! 프로그램을 종료합니다.")

# 모터 정지 후 종료
for dxl_id in DXL_IDS:
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, 0)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()
print("프로그램 종료 완료!")
