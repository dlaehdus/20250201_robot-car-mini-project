"""
이번 코드는 인텔 리얼센스(Intel RealSense) 뎁스 카메라와 YOLOv8 인공지능 모델을 결합하여, 
화면 속 객체의 종류를 식별함과 동시에 해당 객체까지의 실제 거리(Depth)를 실시간으로 측정하는 지능형 비전 스크립트입니다.

기술적으로는 카메라로부터 입력받는 컬러 영상과 거리 데이터 프레임을 동시에 처리하는데,
특히 거리 데이터의 정밀도를 높이기 위해 공간 필터(Spatial Filter)와 시간 필터(Temporal Filter)를 적용하여 노이즈를 제거하는 전처리 과정을 거칩니다. 
인식 단계에서는 YOLOv8 모델이 탐지한 객체 영역(Bounding Box) 내 픽셀들의 거리값 중 중간값(Median)을 산출하여, 
물체까지의 가장 신뢰도 높은 물리적 거리를 미터(m) 단위로 계산해냅니다.

최종적으로 화면에는 탐지된 사물의 이름과 계산된 거리 정보가 실시간으로 표시되며, 이
는 단순히 사물을 보는 것을 넘어 로봇이 주변 환경과의 거리를 입체적으로 인지하게 해줍니다. 
코드 말단에는 객체와의 거리가 0.5m 이하로 근접할 경우를 대비한 조건문이 포함되어 있어, 
향후 로봇의 자율 주행 시 충돌 방지 및 안전 제동 시스템의 핵심 알고리즘으로 활용될 수 있는 구조를 갖추고 있습니다.
"""

import cv2
import numpy as np
import pyrealsense2 as rs #pip install pyrealsense2
from ultralytics import YOLO #pip install ultralytics

import msvcrt

#yolo 임포트
model = YOLO('yolov8s.pt')

#리얼센스 초기 설정 과정 (파이프라인 ,config)
pipeline = rs.pipeline() #rs 그자체로 카메라 불러옴
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

#cm에서 m변환 위해서
depth_scale = 0.0010000000474974513 #depth_image * depth_scale을 통해 cm를 m로 변환

#이미지 필터
spatial = rs.spatial_filter() #이미지를 부드럽게 해주는 필터
temporal = rs.temporal_filter() #이미지를 부드럽게 해주는 필터 (spatial과는 다르게 calculating multiple frames라고 나와있음), 굳이 안해도됨

#동작 시작
try:
    while True:
        #색깔 프레임, 거리 프레임 얻어오기
        frames = pipeline.wait_for_frames() 
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        #수치 행렬화
        color_image = np.asanyarray(color_frame.get_data()) #ndarray(n차원행렬)로 변환, 색깔 프레임의 데이터를 n차원행렬화
        depth_image = np.asanyarray(depth_frame.get_data()) #ndarray(n차원행렬)로 변환, 이번에 뎁스에 대한 프레임을 n차원행렬화, 이렇게 수치화를 시켜야 속도가 향상된다.

        #거리 프레임에 필터 적용
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())

        #cm to m 변환
        depth_image = depth_image * depth_scale #센티미터를 미터화

        #원하는 클래스만 얻어오도록 설정, yolo에 실시간 감지 트리거해주는 단계
        wanted_classes = [i for i in range(0,80) if i != 0]
        #unwanted_classes = 0
        results = model(source=color_image, classes = wanted_classes)
        #results = model(source=color_image, classes = 0)

        

        #본격적인 영상 처리 작업
        for result in results: #영상처리에서 이 문장은 필수적
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                class_id = box.cls[0].cpu().numpy()

                if confidence < 0.5:
                    continue  # 신뢰도가 0.5보다 작으면 이 조건문에서 (계속)고립되어 있고, 0.5 이상이면 밑의 명령들을 실행한다. 

                #물체와의 거리를 계산
                object_depth = np.median(depth_image[y1:y2, x1:x2]) #np.median은 객체의 거리에 대한 중간값 
                #label = f"{object_depth:.2f}m"
                #object_name = f"{model.names[int(class_id)]}m"
                depth_object_name = f"{model.names[int(class_id)]}, {object_depth:.2f}m"

                #사각형 처리
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)

                #텍스트 기입 (오브젝트 이름과 그 거리)
                cv2.putText(color_image, depth_object_name, (x1, y1-10), cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
                
                key = msvcrt.getch()
                #거리가 0.8m 이하이면 아래의 명령문 수행
                if object_depth <= 0.5:
                    key in ('\x7f')

                #필요할 경우 터미널에 오브젝트 이름과 그 거리 출력
                #print(f"{model.names[int(class_id)]}: {object_depth:.2f}m")

        #이미지 디스플레잉
        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
#중단
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
