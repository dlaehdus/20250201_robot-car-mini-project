"""
가장 눈에 띄는 변화는 조향각의 자동 복귀 기능입니다. 
기존에는 a나 d 키를 눌러 바퀴를 꺾으면 그 각도가 그대로 유지되었으나, 
이번 코드에서는 STEERING_DECAY라는 감쇠값을 도입했습니다. 
조종자가 키보드에서 손을 떼는 순간, 꺾여 있던 바퀴가 마치 실제 자동차 핸들이 돌아오듯 서서히 중앙(0°)으로 복귀합니다. 
이는 고속 주행 시 직진 안정성을 높여주며, 조종자가 매번 반대 키를 눌러 수평을 맞출 필요가 없게 만들어 조작 편의성을 극대화합니다.

또한, 서보 모터의 제어 방식도 더욱 정교해졌습니다. 
steering_to_position 함수를 통해 -45°에서 45° 사이의 조향 범위를 다이나믹셀의 중앙값인 2047(0°)을 기준으로 512 단위의 정밀한 디지털 수치로 매핑했습니다. 이
를 통해 아크만 기하학으로 계산된 좌우 바퀴의 미세하게 다른 회전 각도를 하드웨어에 정확히 전달합니다.

결과적으로 이 코드는 "물리적 회전 원리(아크만 스티어링)"와 "심리적 조작 편의성(자동 복원)"을 결합한 주행 시스템입니다.
비록 이번 단계에서는 카메라를 활용한 AI 기능이 잠시 제외되었지만, 로봇이 물리적으로 가장 완벽하게 코너를 돌 수 있는 '최적의 하드웨어 제어 알고리즘'을 완성했다는 데 큰 의의가 있습니다.
"""


import math
import threading
import time
from dynamixel_sdk import *
from pynput import keyboard

# ==========================================
#  글로벌 변수 및 제어 파라미터
# ==========================================
forward_velocity = 0      # 전진/후진 (양수: 전진, 음수: 후진)
steering_angle = 0        # 조향각 (양수: 우회전, 음수: 좌회전); 범위: -45 ~ 45 (°)
exit_flag = False         # 프로그램 종료 플래그
keys_pressed = set()      # 키보드 입력 상태
emergency_stop = False    # (현재 카메라가 없으므로 사용되지 않음)

# 조향각이 누르지 않을 때 서서히 0으로 복귀할 감쇠값 (°)
STEERING_DECAY = 10

# 차량 치수 (단위: mm)
CAR_WIDTH = 305.96   # 차체 폭
CAR_LENGTH = 365.5   # 차체 길이

# ==========================================
#  Dynamixel (모터) 설정
# ==========================================
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'

# 모터 ID 그룹
VELOCITY_IDS = [1, 2, 3, 4]   # 전진/후진 제어
STEERING_IDS = [5, 6]         # 조향 제어 (가정: 5번=왼쪽, 6번=오른쪽)

# 속도 제어 관련 주소 및 모드
ADDR_TORQUE_ENABLE_VELOCITY = 64
ADDR_OPERATING_MODE_VELOCITY = 11
ADDR_GOAL_VELOCITY = 104
VELOCITY_MODE = 1

# 조향 제어 관련 주소 및 모드
ADDR_TORQUE_ENABLE_STEERING = 64
ADDR_OPERATING_MODE_STEERING = 11
ADDR_GOAL_POSITION = 116
POSITION_MODE = 3

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# 제어 파라미터
VELOCITY_STEP = 4      # 속도 증감 단위
VELOCITY_MAX = 200
VELOCITY_MIN = -200
STEERING_STEP = 5      # 조향각 증감 단위 (°)
STEERING_MAX = 45      # 최대 우측 회전 (°)
STEERING_MIN = -45     # 최대 좌측 회전 (°)

# 포트 및 패킷 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("포트를 열 수 없습니다.")
    exit(1)

if not portHandler.setBaudRate(BAUDRATE):
    print("보드레이트 설정 실패")
    exit(1)

# 속도 제어 모터 초기화
for dxl_id in VELOCITY_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE_VELOCITY, VELOCITY_MODE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_ENABLE)

# 조향 모터 초기화
for dxl_id in STEERING_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE_STEERING, POSITION_MODE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_STEERING, TORQUE_ENABLE)

# 모터의 초기 조향 위치를 2047로 설정 (중앙)
packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[0], ADDR_GOAL_POSITION, 2047)
packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[1], ADDR_GOAL_POSITION, 2047)

# ==========================================
#  서보 위치 변환 함수 (조향)
# ==========================================
def steering_to_position(angle):
    """
    입력 angle (°)가 -45 ~ 45 범위일 때,
    0° → 2047, 45° → 2047+512, -45° → 2047-512로 매핑
    """
    return 2047 + int((angle / 45.0) * 512)

# ==========================================
#  키보드 입력 처리 (pynput)
# ==========================================
def on_press(key):
    global exit_flag
    if key == keyboard.Key.backspace:
        exit_flag = True
    else:
        try:
            keys_pressed.add(key.char.lower())
        except AttributeError:
            keys_pressed.add(str(key))

def on_release(key):
    try:
        k = key.char.lower()
    except AttributeError:
        k = str(key)
    if k in keys_pressed:
        keys_pressed.remove(k)

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()

# ==========================================
#  제어 변수 업데이트 스레드 (키보드 입력)
# ==========================================
def update_controls():
    global forward_velocity, steering_angle
    while not exit_flag:
        # 전진/후진: w/s
        if 'w' in keys_pressed:
            forward_velocity = min(forward_velocity + VELOCITY_STEP, VELOCITY_MAX)
        if 's' in keys_pressed:
            forward_velocity = max(forward_velocity - VELOCITY_STEP, VELOCITY_MIN)
        # 우측 회전: d 키 (양수 증가)
        if 'd' in keys_pressed:
            steering_angle = min(steering_angle + STEERING_STEP, STEERING_MAX)
        # 좌측 회전: a 키 (음수 감소)
        if 'a' in keys_pressed:
            steering_angle = max(steering_angle - STEERING_STEP, STEERING_MIN)
        # d, a 키가 눌리지 않으면 서서히 0으로 복귀
        if 'd' not in keys_pressed and 'a' not in keys_pressed:
            if steering_angle > 0:
                steering_angle = max(steering_angle - STEERING_DECAY, 0)
            elif steering_angle < 0:
                steering_angle = min(steering_angle + STEERING_DECAY, 0)
        # r 키: 정지
        if 'r' in keys_pressed:
            forward_velocity = 0
        time.sleep(0.05)

control_thread = threading.Thread(target=update_controls, daemon=True)
control_thread.start()

print("제어 시작:\n  - w/s: 전진/후진\n  - a/d: 좌/우 회전 (누르는 동안 조향각이 증가하며, 누르지 않으면 서서히 0으로 복귀)\n  - r: 정지\n  - 백스페이스: 종료")

# ==========================================
#  메인 제어 루프 (모터 제어 및 아크만 스티어링 적용)
# ==========================================
try:
    while not exit_flag:
        # 속도 제어: 좌우 모터에 교대로 전진/후진 명령 전달
        for i, dxl_id in enumerate(VELOCITY_IDS):
            target_velocity = forward_velocity if i % 2 == 0 else -forward_velocity
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, target_velocity)
        
        # 아크만 스티어링 계산
        if steering_angle == 0:
            left_motor_angle = 0
            right_motor_angle = 0
        else:
            inner_angle = abs(steering_angle)  # 내측 각도 (절댓값, 최대 45°)
            inner_rad = math.radians(inner_angle)
            # R = L / tan(δ_inner) + (T/2)
            R = CAR_LENGTH / math.tan(inner_rad) + (CAR_WIDTH / 2)
            # 외측 각: δ_outer = arctan(L / (R + T/2))
            outer_rad = math.atan(CAR_LENGTH / (R + CAR_WIDTH / 2))
            outer_angle = math.degrees(outer_rad)
            if steering_angle > 0:
                # 우측 회전: 오른쪽 모터(내측)는 inner_angle, 왼쪽(외측)는 outer_angle
                right_motor_angle = inner_angle
                left_motor_angle = outer_angle
            else:
                # 좌측 회전: 왼쪽 모터(내측)는 inner_angle, 오른쪽(외측)는 outer_angle (부호 반전)
                left_motor_angle = -inner_angle
                right_motor_angle = -outer_angle

        # 서보 명령 값 변환: 중앙 2047 기준 ±45°에 대해 ±512 offset
        left_target_position = steering_to_position(left_motor_angle)
        right_target_position = steering_to_position(right_motor_angle)
        # (예시: STEERING_IDS[0]에 오른쪽 모터, STEERING_IDS[1]에 왼쪽 모터로 전송)
        packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[0], ADDR_GOAL_POSITION, right_target_position)
        packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[1], ADDR_GOAL_POSITION, left_target_position)
        
        print("속도: {:>3}   왼쪽 모터: {:>3.1f}°   오른쪽 모터: {:>3.1f}°   (steering_angle: {:>3.1f}°)".format(
            forward_velocity, left_motor_angle, right_motor_angle, steering_angle), end='\r')
        time.sleep(0.01)
except KeyboardInterrupt:
    exit_flag = True
finally:
    # 속도 모터 정지 및 토크 비활성화
    for dxl_id in VELOCITY_IDS:
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, 0)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_DISABLE)
    # 조향 모터 정지 및 토크 비활성화 (중앙인 2047로 복귀)
    packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[0], ADDR_GOAL_POSITION, 2047)
    packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[1], ADDR_GOAL_POSITION, 2047)
    packetHandler.write1ByteTxRx(portHandler, STEERING_IDS[0], ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, STEERING_IDS[1], ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    portHandler.closePort()
    print("\n프로그램 종료 완료!")
