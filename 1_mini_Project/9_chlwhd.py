"""
이번 코드는 로봇의 물리적 구동을 담당하는 '아크만 스티어링(Ackermann Steering)' 시스템과 
주변 상황을 인식하는 'AI 비전 감지' 기술을 하나의 알고리즘으로 완벽하게 통합한 최종 진화형 제어 스크립트입니다.

가장 큰 변화는 로봇이 실제 자동차와 동일한 주행 역학을 가지게 되었다는 점입니다.
차량의 폭과 길이를 계산식에 넣어 회전 시 안쪽 바퀴와 바깥쪽 바퀴의 각도를 다르게 설정하는 아크만 기하학을 적용하여 코너링 시의 마찰과 무리를 최소화했습니다.
여기에 사용자가 키보드에서 손을 떼면 핸들이 자동으로 중앙으로 돌아오는 조향 복원력(Steering Decay) 기능까지 더해져,
실제 운전을 하는 것과 같은 직관적이고 부드러운 조작감을 제공합니다.

시스템 내부적으로는 세 개의 스레드가 독립적이면서도 유기적으로 작동합니다.
첫 번째 스레드는 사용자의 키 입력을 실시간으로 감시하고, 두 번째 스레드는 리얼센스 카메라와 YOLOv8 모델을 통해 전방의 장애물을 식별하고 거리를 계산합니다. 
만약 장애물이 0.5m 이내로 근접하면 세 번째인 메인 제어 루프에서 사용자의 전진 명령을 무시하고 강제로 멈추는 지능형 긴급 제동(Emergency Brake)을 수행합니다.

결국 이 코드는 하드웨어의 물리적 한계를 수학적 공식으로 극복하고, 
인공지능의 시각 능력을 안전 장치로 결합한 반자율 주행 시스템의 표준 모델이라고 할 수 있습니다.
이를 통해 로봇은 단순한 이동 수단을 넘어 주변 환경을 스스로 판단하고 안전하게 기동할 수 있는 지능형 모빌리티로 거듭났습니다.
"""

import math
import threading
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from dynamixel_sdk import *
from pynput import keyboard

# ==========================================
# 글로벌 변수 및 제어 파라미터
# ==========================================
forward_velocity = 0        # 전진/후진 (양수: 전진, 음수: 후진)
steering_angle = 0          # 조향각 (양수: 우회전, 음수: 좌회전); 범위: -45 ~ 45 (°)
exit_flag = False           # 프로그램 종료 플래그
keys_pressed = set()        # 키보드 입력 상태
emergency_stop = False      # 객체 감지에 따른 긴급 정지 (object_depth <= 0.5m 시 True)

# 조향각이 누르지 않을 때 서서히 0으로 복귀할 감쇠값 (°)
STEERING_DECAY = 10

# 차량 치수 (단위: mm)
CAR_WIDTH = 305.96          # 차체 폭
CAR_LENGTH = 365.5          # 차체 길이

# ==========================================
# Dynamixel (모터) 설정
# ==========================================
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM1'

# 모터 ID 그룹
VELOCITY_IDS = [1, 2, 3, 4]  # 전진/후진 제어
STEERING_IDS = [5, 6]        # 조향 제어 (가정: 5번=왼쪽, 6번=오른쪽)

# 속도 제어 주소 및 모드
ADDR_TORQUE_ENABLE_VELOCITY = 64
ADDR_OPERATING_MODE_VELOCITY = 11
ADDR_GOAL_VELOCITY = 104
VELOCITY_MODE = 1

# 조향 제어 주소 및 모드
ADDR_TORQUE_ENABLE_STEERING = 64
ADDR_OPERATING_MODE_STEERING = 11
ADDR_GOAL_POSITION = 116
POSITION_MODE = 3

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# 제어 파라미터
VELOCITY_STEP = 4           # 속도 증감 단위
VELOCITY_MAX = 200
VELOCITY_MIN = -200
STEERING_STEP = 5           # 조향각 증감 단위 (°)
STEERING_MAX = 45           # 최대 우측 회전 (°)
STEERING_MIN = -45          # 최대 좌측 회전 (°)

# 서보 위치 변환 함수 (조향)
def steering_to_position(angle):
    """
    입력 angle (°)가 -45 ~ 45 범위일 때,
    0° → 2047, 45° → 2047+512, -45° → 2047-512로 매핑
    """
    return 2047 + int((angle / 45.0) * 512)

# 다이나믹셀 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
if not portHandler.openPort():
    print("포트를 열 수 없습니다.")
    exit(1)
if not portHandler.setBaudRate(BAUDRATE):
    print("보드레이트 설정 실패")
    exit(1)
# 속도 모터 초기화
for dxl_id in VELOCITY_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE_VELOCITY, VELOCITY_MODE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_ENABLE)
# 조향 모터 초기화
for dxl_id in STEERING_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE_STEERING, POSITION_MODE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_STEERING, TORQUE_ENABLE)
# 초기 조향 위치를 중앙(2047)로 설정
packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[0], ADDR_GOAL_POSITION, 2047)
packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[1], ADDR_GOAL_POSITION, 2047)

# ==========================================
# RealSense 및 YOLO 설정
# ==========================================
# YOLO 모델 로드
model = YOLO('yolov8s.pt')
# Realsense 파이프라인 및 스트림 설정
pipeline = rs.pipeline()
config_rs = rs.config()
config_rs.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config_rs.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config_rs)
# Depth scale (cm에서 m로 변환)
depth_scale = 0.0010000000474974513
# 이미지 필터 (옵션)
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()

# ==========================================
# 키보드 입력 처리 (pynput)
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

keyboard.Listener(on_press=on_press, on_release=on_release).daemon = True
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()

# ==========================================
# 제어 변수 업데이트 스레드 (키보드 입력 및 긴급 정지 반영)
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
        # 만약 emergency_stop가 활성화되면 전진 속도 0으로 강제
        if emergency_stop:
            forward_velocity = 0
        time.sleep(0.05)

control_thread = threading.Thread(target=update_controls, daemon=True)
control_thread.start()

# ==========================================
# 객체 감지 스레드 (RealSense + YOLO)
# ==========================================
def object_detection_thread():
    global emergency_stop, exit_flag
    while not exit_flag:
        # 프레임 획득
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # 필터 적용
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = depth_image * depth_scale  # 센티미터 -> 미터

        # YOLO 객체 감지 (원하는 클래스: 배경(0) 제외)
        wanted_classes = [i for i in range(0,80) if i != 0]
        results = model(source=color_image, classes=wanted_classes, verbose=False)

        emergency = False
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                class_id = box.cls[0].cpu().numpy()
                if confidence < 0.5:
                    continue
                object_depth = np.median(depth_image[y1:y2, x1:x2])
                label = f"{model.names[int(class_id)]}, {object_depth:.2f}m"
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                cv2.putText(color_image, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
                # 만약 물체가 0.5m 이하이면 긴급 정지 활성화
                if object_depth <= 0.5:
                    emergency = True
        emergency_stop = emergency

        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_flag = True

# 객체 감지 스레드를 시작 (데몬 스레드)
od_thread = threading.Thread(target=object_detection_thread, daemon=True)
od_thread.start()

print("프로그램 시작:")
print("  [RealSense+YOLO]  -> q: 종료")
print("  [모터 제어]  -> w/s: 전진/후진, a/d: 좌/우 회전, r: 정지, backspace: 종료")

# ==========================================
# 메인 제어 루프 (다이나믹셀 모터 제어 및 아크만 스티어링 적용)
# ==========================================
try:
    while not exit_flag:
        # 속도 제어: 좌우 모터에 교대로 전진/후진 명령 전송
        for i, dxl_id in enumerate(VELOCITY_IDS):
            target_velocity = forward_velocity if i % 2 == 0 else -forward_velocity
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, target_velocity)
        
        # 아크만 스티어링 계산 (steering_angle의 절댓값 사용)
        if steering_angle == 0:
            left_motor_angle = 0
            right_motor_angle = 0
        else:
            inner_angle = abs(steering_angle)
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
        # 예시: STEERING_IDS[0]에 오른쪽 모터, STEERING_IDS[1]에 왼쪽 모터 전송
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
    # 조향 모터 중앙 복귀 (2047)
    packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[0], ADDR_GOAL_POSITION, 2047)
    packetHandler.write4ByteTxRx(portHandler, STEERING_IDS[1], ADDR_GOAL_POSITION, 2047)
    packetHandler.write1ByteTxRx(portHandler, STEERING_IDS[0], ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, STEERING_IDS[1], ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    portHandler.closePort()
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\n프로그램 종료 완료!")
