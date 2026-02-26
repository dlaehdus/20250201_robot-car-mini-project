"""
이번 코드는 바퀴 구동을 위한 속도 제어 방식에서 벗어나, 
모터의 회전 각도를 정밀하게 조절하는 위치 제어(Position Control) 기반의 카메라 팬/틸트 조작 스크립트입니다.

다이나믹셀 5번과 6번 모터의 동작 모드를 '3(Position Mode)'으로 설정하여 0°에서 360° 사이의 특정 지점으로 정교하게 이동시킨 뒤 그 자리에 고정(Hold)시키는 것이 핵심입니다.
사용자가 a나 d 키를 누르면 시스템은 현재 모터의 위치 값을 실시간으로 읽어온 뒤,
이를 기준으로 ±10°씩 목표 각도를 계산하여 명령을 내리는 증분 제어 방식을 채택하고 있습니다.

내부적으로는 사람이 사용하는 '도(Degree)' 단위와 모터가 인식하는 '0~4095' 사이의 디지털 수치를 상호 변환하는 매핑 로직을 포함하며,
-180°에서 180° 사이의 가동 범위를 설정하여 기계적 파손을 방지합니다. 
결과적으로 이 코드는 로봇의 상단에 부착된 카메라의 시야각을 세밀하게 조정하여 특정 객체를 정면으로 포착하기 위한 방향 제어기로 활용됩니다.
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
DXL_IDS = [5, 6]  # 5번, 6번 다이나믹셀 ID

# 다이나믹셀 주소값
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11  # 동작 모드
ADDR_GOAL_POSITION = 116  # 목표 위치 (4Byte)
ADDR_PRESENT_POSITION = 132  # 현재 위치 (4Byte)

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
POSITION_MODE = 3  # 각도 모드

# 포트 및 패킷 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# 포트 열기
if not portHandler.openPort():
    print("Failed to open the port")
    exit()

# 보드레이트 설정
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    exit()

# 다이나믹셀 설정 (각도 모드 + 토크 활성화)
for dxl_id in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, POSITION_MODE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# 현재 위치 읽기 함수
def get_current_position(dxl_id):
    pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"통신 실패: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error:
        print(f"오류 발생: {packetHandler.getRxPacketError(dxl_error)}")
    return pos

# 각도를 다이나믹셀 값(0~4095)으로 변환
def angle_to_position(angle):
    return int((angle / 360.0) * 4095)

# 다이나믹셀 값을 각도로 변환
def position_to_angle(position):
    return (position / 4095.0) * 360.0

try:
    while True:
        key = getch()

        if key == '\x7f':  # Backspace (ASCII 코드 127)
            print("프로그램 종료")
            break

        for dxl_id in DXL_IDS:
            current_pos = get_current_position(dxl_id)
            current_angle = position_to_angle(current_pos)

            if key == 'd':  # +1도 회전 (최대 90도까지)
                target_angle = min(180, current_angle + 10)
            elif key == 'a':  # -1도 회전 (최소 -90도까지)
                target_angle = max(-180, current_angle - 10)
            else:
                continue

            target_pos = angle_to_position(target_angle)
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, target_pos)
            print(f"모터 {dxl_id}: {current_angle:.1f}° → {target_angle:.1f}°")

except KeyboardInterrupt:
    print("\n프로그램 종료!")

# 모터 정지 후 종료
for dxl_id in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()
