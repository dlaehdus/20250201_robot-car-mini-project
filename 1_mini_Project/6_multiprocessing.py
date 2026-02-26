"""
이번 코드는 지금까지 개별적으로 구현했던 AI 객체 인식, 실시간 거리 측정, 멀티스레드 모터 제어 기능을 하나로 통합한 '지능형 자율 안전 주행 시스템'의 완성본입니다.

가장 큰 특징은 로봇의 인지(Vision), 판단(AI), 제어(Motor)가 독립적인 3중 스레드 구조로 설계되었다는 점입니다. 
'키보드 리스너 스레드'는 조종사의 주행 의도를 파악하고, 
'AI 비전 스레드'는 리얼센스 카메라를 통해 전방 장애물과의 거리를 0.01초 단위로 계산하며, 
'메인 제어 루프'는 이 정보들을 종합하여 최종적으로 모터에 구동 명령을 내립니다.

이 시스템의 핵심은 지능형 긴급 제동(Emergency Brake) 로직입니다. 
조종사가 w 키를 눌러 전진하더라도, YOLO 모델과 뎁스 카메라가 전방 0.5m 이내에서 장애물을 감지하면 emergency_stop 플래그가 활성화되어 전진 속도를 강제로 0으로 고정합니다. 
이때 화면에는 빨간색 경고 문구가 표시되어 사용자에게 위험 상황임을 직관적으로 알립니다.

결과적으로 이 코드는 사용자의 원격 조종 편의성을 유지하면서도, 
하드웨어 스스로 주변 환경을 분석하여 충돌을 방지하는 반자율 주행(Semi-Autonomous) 플랫폼의 형태를 갖추고 있습니다.
이는 향후 단순한 조종을 넘어 특정 목적지를 스스로 찾아가거나 위험 요소를 피하는 완전 자율 주행 로봇으로 발전하기 위한 완벽한 기초 기반이 됩니다.
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import threading
import time
from dynamixel_sdk import *
from pynput import keyboard

# ==========================================
#  글로벌 변수 및 제어 파라미터
# ==========================================
# 모터 제어 관련 변수
forward_velocity = 0      # 전진/후진 (양수: 전진, 음수: 후진)
steering_angle = 0        # 조향 각 (양수: 우회전, 음수: 좌회전)
exit_flag = False         # 프로그램 종료 플래그
keys_pressed = set()      # 실제 키 입력 상태
emergency_stop = False    # 객체 감지에 따른 긴급 정지 (True면 전진 명령 무시)

def angle_to_position(angle):
    """0~360° 범위를 다이나믹셀의 포지션 값(0~4095)으로 변환"""
    return int((angle / 360.0) * 4095)
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
STEERING_IDS = [5, 6]         # 좌/우 조향

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
    # 백스페이스 키로 종료: 백스페이스는 keyboard.Key.backspace
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
        # 수동 조향 제어
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
        results = model(source=color_image, classes=wanted_classes, verbose = False)

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
        for i, dxl_id in enumerate(VELOCITY_IDS):
            target_velocity = forward_velocity if i % 2 == 0 else -forward_velocity
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, target_velocity)
        target_position = angle_to_position(steering_angle)
        for dxl_id in STEERING_IDS:
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, target_position)
        
        print("속도: {:>3}   조향각: {:>3}".format(forward_velocity, steering_angle), end='\r')
        time.sleep(0.01)
except KeyboardInterrupt:
    exit_flag = True

finally:
    for dxl_id in VELOCITY_IDS:
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, 0)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_DISABLE)
    for dxl_id in STEERING_IDS:
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, angle_to_position(0))
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    portHandler.closePort()
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\n프로그램 종료 완료!")
