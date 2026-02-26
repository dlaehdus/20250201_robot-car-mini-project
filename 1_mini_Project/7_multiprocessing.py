"""
이번 코드는 기존의 단순 조향 방식을 넘어, 
실제 자동차의 물리적 회전 원리인 '아크만 스티어링(Ackermann Steering)' 기하학을 적용한 고도화된 주행 제어 스크립트입니다.

가장 핵심적인 차이점은 좌우 두 개의 조향 모터가 서로 다른 각도로 움직인다는 점입니다. 
이전까지는 좌우 바퀴를 단순히 같은 각도로 꺾었다면, 이번 버전에서는 차량의 폭(CAR_WIDTH)과 길이(CAR_LENGTH) 데이터를 바탕으로 수학적 계산(삼각함수)을 수행합니다. 
회전 시 안쪽 바퀴는 더 크게 꺾고 바깥쪽 바퀴는 완만하게 꺾이도록 실시간으로 계산하여, 
타이어의 미끄러짐을 최소화하고 훨씬 매끄럽고 안정적인 코너링을 구현했습니다.

기술적으로는 조향 입력값(steering_angle)을 내측 바퀴의 기준으로 삼고,
아크만 공식을 통해 외측 바퀴의 최적 각도를 산출하는 로직이 추가되었습니다.
여기에 기존의 YOLO 기반 실시간 객체 인식과 리얼센스 뎁스 센서를 이용한 긴급 제동 기능이 그대로 유지되어,
물리적으로 완벽한 조향 성능과 지능형 안전 시스템이 결합된 완성형 자동차 주행 메커니즘을 보여주고 있습니다.
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import threading
import time
from dynamixel_sdk import *
from pynput import keyboard
import math  # 삼각함수 사용을 위한 math 모듈 추가

# ==========================================
#  글로벌 변수 및 제어 파라미터
# ==========================================
# 모터 제어 관련 변수
forward_velocity = 0      # 전진/후진 (양수: 전진, 음수: 후진)
steering_angle = 0        # 키 입력에 따른 “내측 조향각” (양수: 우회전, 음수: 좌회전)
exit_flag = False         # 프로그램 종료 플래그
keys_pressed = set()      # 실제 키 입력 상태
emergency_stop = False    # 객체 감지에 따른 긴급 정지 (True면 전진 명령 무시)

def angle_to_position(angle):
    """0~360° 범위를 다이나믹셀의 포지션 값(0~4095)으로 변환"""
    return int((angle / 360.0) * 4095)

# 차량 치수 (단위: mm)
CAR_WIDTH = 305.96   # 차체 폭
CAR_LENGTH = 365.5   # 차체 길이

# ==========================================
#  Realsense & YOLO 객체 감지 설정
# ==========================================
# YOLO 모델 불러오기 (yolov8s.pt 파일 필요)
model = YOLO('yolov8s.pt')

# Realsense 파이프라인 및 스트림 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# depth scale (센티미터를 미터로 변환)
depth_scale = 0.0010000000474974513

# 이미지 필터 (옵션)
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

# ==========================================
#  Dynamixel (모터) 설정
# ==========================================
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'

# 모터 ID 그룹 (속도 제어와 조향 제어)
VELOCITY_IDS = [1, 2, 3, 4]   # 전진/후진
STEERING_IDS = [5, 6]         # 조향 (가정: STEERING_IDS[0]=좌측, STEERING_IDS[1]=우측)

# 주소 및 모드 (속도 제어)
ADDR_TORQUE_ENABLE_VELOCITY = 64
ADDR_OPERATING_MODE_VELOCITY = 11
ADDR_GOAL_VELOCITY = 104
VELOCITY_MODE = 1

# 주소 및 모드 (각도 제어)
ADDR_TORQUE_ENABLE_STEERING = 64
ADDR_OPERATING_MODE_STEERING = 11
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
POSITION_MODE = 3

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# 제어 파라미터
VELOCITY_STEP = 4      # 속도 증감 단위
VELOCITY_MAX = 200
VELOCITY_MIN = -200
STEERING_STEP = 5     # 조향 각 증감 단위 (°)
STEERING_MAX = 90
STEERING_MIN = -90

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

# ==========================================
#  키보드 입력 처리 (pynput)
# ==========================================
def on_press(key):
    global exit_flag
    # 백스페이스 키로 종료: keyboard.Key.backspace
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

# 키 리스너 시작 (데몬 스레드)
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()

# ==========================================
#  제어 변수 업데이트 스레드 (키 상태 + 긴급 정지)
# ==========================================
def update_controls():
    global forward_velocity, steering_angle
    while not exit_flag:
        # 긴급 정지 활성화 시 전진 속도 강제 0
        if emergency_stop:
            forward_velocity = 0
        else:
            # 수동 전진/후진
            if 'w' in keys_pressed:
                forward_velocity = min(forward_velocity + VELOCITY_STEP, VELOCITY_MAX)
            if 's' in keys_pressed:
                forward_velocity = max(forward_velocity - VELOCITY_STEP, VELOCITY_MIN)
        # 수동 조향 제어 (내측 조향각 입력)
        if 'd' in keys_pressed:
            steering_angle = min(steering_angle + STEERING_STEP, STEERING_MAX)
        if 'a' in keys_pressed:
            steering_angle = max(steering_angle - STEERING_STEP, STEERING_MIN)
        # 수동 정지 명령
        if 'r' in keys_pressed:
            forward_velocity = 0
        time.sleep(0.05)

control_thread = threading.Thread(target=update_controls, daemon=True)
control_thread.start()

# ==========================================
#  객체 감지 스레드 (Realsense + YOLO)
# ==========================================
def object_detection_loop():
    global emergency_stop, exit_flag
    while not exit_flag:
        # 프레임 획득
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # 프레임 배열 변환
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # 필터 적용
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())
        # m 단위 변환
        depth_image = depth_image * depth_scale

        # YOLO를 통한 객체 감지 (배경 클래스 0 제외)
        wanted_classes = [i for i in range(0,80) if i != 0]
        results = model(source=color_image, classes=wanted_classes, verbose=False)

        local_stop = False

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                class_id = box.cls[0].cpu().numpy()

                if confidence < 0.5:
                    continue

                # 영역 중간값으로 물체 거리 계산
                object_depth = np.median(depth_image[y1:y2, x1:x2])
                label_text = f"{model.names[int(class_id)]}, {object_depth:.2f}m"

                cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                cv2.putText(color_image, label_text, (x1, y1-10),
                            cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)

                # 물체가 0.5 m 이하이면 긴급 정지 활성화
                if object_depth <= 0.5:
                    local_stop = True

        emergency_stop = local_stop

        if emergency_stop:
            cv2.putText(color_image, "EMERGENCY STOP", (50,50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)

        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_flag = True

detection_thread = threading.Thread(target=object_detection_loop, daemon=True)
detection_thread.start()

print("제어 시작: \n  - w/s: 전진/후진\n  - a/d: 좌/우 회전\n  - r: 정지 (수동 또는 긴급정지)\n  - 백스페이스: 종료")

# ==========================================
#  메인 제어 루프 (모터 제어)
# ==========================================
try:
    while not exit_flag:
        # 속도 제어: 좌우 모터에 교대로 전진/후진 명령 전달
        for i, dxl_id in enumerate(VELOCITY_IDS):
            target_velocity = forward_velocity if i % 2 == 0 else -forward_velocity
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, target_velocity)
        
        # 아크만 스티어링 적용
        # steering_angle 변수는 내측 조향각 입력 (°)를 나타냅니다.
        if steering_angle == 0:
            left_motor_angle = 0
            right_motor_angle = 0
        else:
            # 내측 각(절댓값)
            inner_angle = abs(steering_angle)
            inner_rad = math.radians(inner_angle)
            # tan(내측각) = L / (R - T/2)  →  R = L/tan(내측각) + T/2
            R = CAR_LENGTH / math.tan(inner_rad) + (CAR_WIDTH / 2)
            # 외측 각: tan(외측각) = L / (R + T/2)
            outer_rad = math.atan(CAR_LENGTH / (R + CAR_WIDTH / 2))
            outer_angle = math.degrees(outer_rad)
            # 회전 방향에 따라 내측/외측 모터 할당
            if steering_angle > 0:
                # 우회전: 오른쪽이 내측, 왼쪽이 외측
                right_motor_angle = inner_angle
                left_motor_angle = outer_angle
            else:
                # 좌회전: 왼쪽이 내측, 오른쪽이 외측
                left_motor_angle = -inner_angle
                right_motor_angle = -outer_angle
        
        # 다이나믹셀에 각도 명령 전송
        # 가정: STEERING_IDS[0] = 좌측 모터, STEERING_IDS[1] = 우측 모터
        left_target_position = angle_to_position(left_motor_angle)
        right_target_position = angle_to_position(right_motor_angle)
        packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[0], ADDR_GOAL_POSITION, left_target_position)
        packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[1], ADDR_GOAL_POSITION, right_target_position)
        
        print("속도: {:>3}   좌각: {:>3.1f}°   우각: {:>3.1f}°".format(forward_velocity, left_motor_angle, right_motor_angle), end='\r')
        time.sleep(0.01)
except KeyboardInterrupt:
    exit_flag = True

finally:
    # 속도 모터 정지 및 토크 비활성화
    for dxl_id in VELOCITY_IDS:
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, 0)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_DISABLE)
    # 조향 모터 정지 및 토크 비활성화 (0도 복귀)
    packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[0], ADDR_GOAL_POSITION, angle_to_position(0))
    packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[1], ADDR_GOAL_POSITION, angle_to_position(0))
    packetHandler.write1ByteTxRx(portHandler, STEERING_IDS[0], ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, STEERING_IDS[1], ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    portHandler.closePort()
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\n프로그램 종료 완료!")
