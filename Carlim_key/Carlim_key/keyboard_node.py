"""
이 코드는 사용자의 키보드 조작을 실제 자동차의 주행 원리에 맞춰 계산해 주는 조종부입니다.

가장 큰 특징은 아커만 기하학이라는 전문적인 설계 공식을 사용하는 것입니다.
자동차가 곡선을 돌 때 안쪽 바퀴와 바깥쪽 바퀴가 그리는 원의 크기가 다르다는 점에 착안하여,
4개 바퀴의 속도와 각도를 각각 다르게 계산합니다. 이를 통해 바퀴가 바닥에 끌리지 않고 매우 부드럽게 회전할 수 있습니다.

또한, 실제 운전과 유사한 조작감을 제공합니다. 
사용자가 키보드 W, A, S, D로 로봇을 조종하다가 손을 떼면, 
바퀴가 서서히 정면을 향해 돌아오고 속도도 자연스럽게 줄어드는 복원 로직이 들어있습니다.

마지막으로, 이렇게 계산된 정밀한 값들을 ROS2 통신망을 통해 앞서 준비한 모터 엔진들에게 전달합니다. 
요약하자면, 사람의 단순한 키 입력을 복잡한 물리 공식으로 계산하여 로봇이 자동차처럼 똑똑하게 움직이게 만드는 핵심 컨트롤러입니다.
"""

import time                                     # 시간 지연 사용 - 키 입력에 대한 시간지연
import threading                                # 쓰레드로 병렬 키 입력 처리
import sys                                      # 벡스페이스 입력시 시스템 종료 기능
import math                                     # 아커만 계산에 필요한 수학 계산
import rclpy                                    # ROS2의 Python 라이브러리, 노드 생성/스핀 등.
from rclpy.node import Node                     # ROS2 노드 클래스 노드를 만들기 위한 기본 클래스
from std_msgs.msg import Float64MultiArray      # ROS2 메시지 타입으로 조향각, 속도등 여러 부동소수점 값 을 전송

'''
로봇 키입력 및 아크만 계산 로직
w - 전진
s - 후진
d - 우회전
a - 좌회전
r - 속도와 각도를 초기상태로
'''
class KeyboardController:
    def __init__(self):
        self.keys_pressed = set()               # 눌린 키 저장하는 집합을 초기화
        self.velocity = 0                       # 초기속도를 0 으로 초기화
        self.steering = 0                       # 초기각도를 0 으로 초기화
        self.robot_exit = False                 # 프로그램 종료 플래그를 False로 초기화

        self.VELOCITY_STEP = 10                 # 키 입력시 인휠모터 속도 변화값
        self.STEERING_STEP = 30                 # 키 입력시 다이나믹셀 각도 변화값

        self.RETURN_VELOCITY = 5                # 키 입력이 없을시 인휠모터 속도 복귀값
        self.RETURN_STEERING = 15               # 키 입력이 없을시 다이나믹셀 각도 복귀값

        self.VELOCITY_MAX = 330                 # 최대 인휠모터 속도
        self.VELOCITY_MIN = -330                # 최소 인휠모터 각도
        self.STEERING_MAX = 60                  # 최대 다이나믹셀 각도
        self.STEERING_MIN = -60                 # 최소 다이나믹셀 각도

        self.ROBOT_WIDTH = 347                  # 로봇 폭
        self.ROBOT_LENGTH = 450                 # 로봇 길이

    def process_key_event(self, event: str):                # 키 이벤트 처리 메서드를 정의, event는 문자열(str) 입력 파라미터
        if event.startswith("press:"):                      # event 문자열이 "press:"로 시작하는지 확인, 만약 맞으면 키 누름(press) 이벤트로 간주
            key = event[len("press:"):].strip().lower()     # event에서 "press:" 부분의 길이(len("press:")=6)를 이용해 그 이후 문자열을 슬라이싱합니다. 예: "press:w" → "w".
            if key == "backspace":                          # 만약 벡스페이스가 키 입력으로 눌리면
                self.robot_exit = True                      # robot_exit를 True 로 변화시키고 프로그램을 종료함
            else:
                self.keys_pressed.add(key)                  # self.keys_pressed 집합(set)에 key를 추가, set은 자동으로 중복을 방지하므로, 같은 키가 여러 번 추가되지 않음
        elif event.startswith("release:"):                  # event가 "release:"로 시작하는지 확인합니다. 맞으면 키 떼기(release) 이벤트로 간주하고 아래 블록을 실행
            key = event[len("release:"):].strip().lower()   # event에서 "release:" 부분(길이 8)을 제거
            self.keys_pressed.discard(key)                  # self.keys_pressed 집합에서 key를 제거, discard()는 키가 없어도 에러를 발생시키지 않습니다. (예: 'w' 제거 → keys_pressed에서 'w' 사라짐.)

    def start(self):                                                                            # update_controls 메서드를 별도의 스레드(쓰레드)로 실행하여, 키 입력을 지속적으로 감지하고 속도/조향 값을 업데이트
        self.control_thread = threading.Thread(target=self.update_controls, daemon=True)        # 스레드 객체 생성: threading 모듈의 Thread 클래스를 사용해 새로운 스레드 객체를 만듬
                                                                                                # self.control_thread: 클래스 변수로 스레드 객체를 저장합니다. 나중에 스레드를 제어(예: join)할 때 사용
                                                                                                # target=self.update_controls: 이 스레드가 실행할 함수(타겟)를 self.update_controls 메서드로 지정
                                                                                                # daemon=True: 데몬 스레드로 설정합니다. 데몬 스레드는 메인 프로그램(메인 스레드)이 종료되면 자동으로 종료됩니다. (False면 메인 종료 후에도 스레드가 계속 동작할 수 있음.)
        self.control_thread.start()                                                             # 스레드 시작: 생성된 스레드 객체의 start() 메서드를 호출
                                                                                                # 이로 인해 target으로 지정된 update_controls 메서드가 별도 스레드에서 실행되기 시작

    def update_controls(self):                                                                  # 키 상태를 지속적으로 모니터링하여 속도와 조향을 조정
        while not self.robot_exit:                                                              # 무한 루프 시작: self.robot_exit가 False인 동안 아래 블록을 반복 실행합니다. (True가 되면 루프 종료, 프로그램 종료 유발. backspace 키로 설정됨.)
            if 'w' in self.keys_pressed:                                                        # self.keys_pressed 집합에 'w' 키가 포함되어 있으면(눌린 상태) 아래 코드를 실행
                self.velocity = min(self.velocity + self.VELOCITY_STEP, self.VELOCITY_MAX)      # 속도 증가: 현재 velocity에 VELOCITY_STEP(10)을 더합니다. min()으로 VELOCITY_MAX(330)을 초과하지 않게 제한
            if 's' in self.keys_pressed:                                                        # 조건 검사: 's' 키가 눌려 있으면 아래 코드를 실행
                self.velocity = max(self.velocity - self.VELOCITY_STEP, self.VELOCITY_MIN)      # 속도 감소: velocity에서 VELOCITY_STEP(10)을 뺍니다. max()으로 VELOCITY_MIN(-330) 이하로 떨어지지 않게 제한
            if 'w' not in self.keys_pressed and 's' not in self.keys_pressed:                   # 조건 검사: 'w'와 's' 둘 다 눌리지 않았으면(전/후진 키 떼짐) 아래 코드를 실행
                if self.velocity > 0:                                                           # 하위 조건: velocity가 양수(전진 중)이면 아래 코드를 실행
                    self.velocity = max(self.velocity - self.RETURN_VELOCITY, 0)                # 속도 복귀: velocity에서 RETURN_VELOCITY(5)를 뺍니다. max()으로 0 이하로 떨어지지 않게 합니다
                elif self.velocity < 0:                                                         # velocity가 음수(후진 중)이면 아래 코드를 실행
                    self.velocity = min(self.velocity + self.RETURN_VELOCITY, 0)                # 속도 복귀: velocity에 RETURN_VELOCITY(5)를 더합니다. min()으로 0 이상으로 올라가지 않게 합니다
            if 'd' in self.keys_pressed:                                                        # 조건 검사: 'd' 키가 눌려 있으면 아래 코드를 실행
                self.steering = min(self.steering + self.STEERING_STEP, self.STEERING_MAX)      # 조향 증가: steering에 STEERING_STEP(30)을 더합니다. min()으로 STEERING_MAX(60)을 초과하지 않습니다
            if 'a' in self.keys_pressed:                                                        # 조건 검사: 'a' 키가 눌려 있으면 아래 코드를 실행
                self.steering = max(self.steering - self.STEERING_STEP, self.STEERING_MIN)      # 조향 감소: steering에서 STEERING_STEP(30)을 뺍니다. max()으로 STEERING_MIN(-60) 이하로 떨어지지 않습니다
            if 'd' not in self.keys_pressed and 'a' not in self.keys_pressed:                   # 조건 검사: 'd'와 'a' 둘 다 눌리지 않았으면 아래 코드를 실행
                if self.steering > 0:                                                           # 하위 조건: steering이 양수(오른쪽 회전 중)이면 아래 코드를 실행
                    self.steering = max(self.steering - self.RETURN_STEERING, 0)                # 조향 복귀: steering에서 RETURN_STEERING(15)를 뺍니다. max()으로 0 이하 방지.
                elif self.steering < 0:                                                         # steering이 음수(왼쪽 회전 중)이면 아래 코드를 실행
                    self.steering = min(self.steering + self.RETURN_STEERING, 0)                # 조향 복귀: steering에 RETURN_STEERING(15)를 더합니다. min()으로 0 이상 방지
            if 'r' in self.keys_pressed:                                                        # 조건 검사: 'r' 키가 눌려 있으면 아래 코드를 실행
                self.velocity = 0                                                               # 속도를 즉시 0으로 설정
            time.sleep(0.05)                                                                    # 0.05초 지연: 루프를 한 번 돌 때마다 0.05초 대기합니다. CPU 과부하 방지와 실시간 처리 속도 조절

    def compute_ackermann(self):
        vel = self.velocity
        steer = self.steering

        if steer == 0:                                              # 만약에 회전하지 않고 직진과 후진만 하는 상황이라면
            wheel_speeds = [vel, vel, vel, vel]                     # 바퀴 속도 리스트 생성: 모든 4개 바퀴(앞왼쪽 FL, 앞오른쪽 FR, 뒤왼쪽 RL, 뒤오른쪽 RR)에 vel 값을 동일하게 설정
            steering_angles = [0.0, 0.0]                            # 조향 각도 리스트 생성: 앞바퀴 2개(왼쪽 left, 오른쪽 right)에 0.0을 설정
        else:                                                       # 회전하는 상황이면 즉 선회상태이면
            direction = 1 if steer > 0 else -1                      # steer가 양수이면 direction=1(오른쪽 선회), 음수이면 -1(왼쪽 선회)으로 설정
            
            # 회전반경 계산 코드
            input_angle = abs(steer)                                # 입력 각도의 절대값을 input_angle에 저장
            inner_angle_rad = math.radians(input_angle)             # input_angle을 라디안 단위로 변환
            R = (self.ROBOT_LENGTH / math.tan(inner_angle_rad)) + (self.ROBOT_WIDTH / 2)                       # 선회 반경 R 계산 (아크만 기하학 공식.)
            outer_front_angle = math.degrees(math.atan(self.ROBOT_LENGTH / (R + self.ROBOT_WIDTH / 2)))      # 외측 앞바퀴 각도 계산: atan 함수로 각도를 구하고 degrees로 변환
            
            # 속도 계산 코드 각속도를 이용함
            w = (vel/R)
            r_inner_front = math.sqrt((R - (self.ROBOT_WIDTH / 2))**2 + (self.ROBOT_LENGTH)**2)
            r_outer_front = math.sqrt((R + (self.ROBOT_WIDTH / 2))**2 + (self.ROBOT_LENGTH)**2)
            r_inner_back = (R - (self.ROBOT_WIDTH / 2))
            r_outer_back = (R + (self.ROBOT_WIDTH / 2))
            
            v_inner_front = (w * r_inner_front)
            v_outer_front = (w * r_outer_front)
            v_inner_back = (w * r_inner_back)
            v_outer_back = (w * r_outer_back)

            if direction > 0:                                       # direction이 양수(오른쪽 선회)이면 아래 코드를 실행
                front_right = input_angle                           # 오른쪽 앞바퀴 각도를 input_angle(내측 각도)로 설정
                front_left = outer_front_angle                      # 왼쪽 앞바퀴 각도를 outer_front_angle(외측 각도)로 설정
            else:                                                   # direction이 음수(왼쪽 선회)이면 아래 코드를 실행
                front_left = input_angle                            # 왼쪽 앞바퀴 각도를 input_angle(내측 각도)로 설정
                front_right = outer_front_angle                     # 오른쪽 앞바퀴 각도를 outer_front_angle(외측 각도)로 설정
                front_left = -front_left                            # 왼쪽 각도를 음수로 변환합니다. (왼쪽 선회 방향 반영.)
                front_right = -front_right                          # 오른쪽 각도를 음수로 변환합니다. (왼쪽 선회 방향 반영.)
            steering_angles = [front_left, front_right]             # 조향 각도 리스트 생성: [왼쪽, 오른쪽] 순서로 저장

            if direction > 0:                                                               # 조건 검사: 오른쪽 선회이면 아래 코드를 실행
                wheel_speeds = [v_inner_front, v_outer_front, v_inner_back, v_outer_back]   # 바퀴 속도 리스트: [내측, 외측, 내측, 외측] 순서로 설정 (FL 내측, FR 외측 등).
            else:                                                                           # 왼쪽 선회이면 아래 코드를 실행
                wheel_speeds = [v_outer_front, v_inner_front, v_outer_back, v_inner_back]   # 바퀴 속도 리스트: [외측, 내측, 외측, 내측] 순서로 설정 (FL 외측, FR 내측 등).
        return wheel_speeds, steering_angles                                                # 반환: wheel_speeds와 steering_angles를 튜플로 반환

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')                                                       # 상위 클래스(Node)의 init 메서드를 호출합니다. 노드 이름을 'keyboard_node'로 설정
        self.speed_publisher = self.create_publisher(Float64MultiArray, '/wheel_speeds', 10)    # /wheel_speeds 토픽으로 Float64MultiArray 메시지를 발행할 퍼블리셔를 생성
                                                                                                # queue_size=10으로 메시지 버퍼 크기 설정. (바퀴 속도 배열을 다른 노드로 전송하기 위함.)
        self.angle_publisher = self.create_publisher(Float64MultiArray, '/steering_angles', 10) # /steering_angles 토픽으로 Float64MultiArray 메시지를 발행할 퍼블리셔를 생성
                                                                                                # queue_size=10. (조향 각도 배열을 다른 노드로 전송하기 위함.)
        self.keyboard_ctrl = KeyboardController()                                               # KeyboardController 클래스의 인스턴스를 생성하고 self.keyboard_ctrl 변수에 저장
        self.keyboard_ctrl.start()                                                              # 생성된 KeyboardController 객체의 start 메서드를 호출
        self.timer = self.create_timer(0.05, self.publish_command)                              # 0.05초 간격으로 self.publish_command 메서드를 호출하는 타이머를 생성

    def publish_command(self):                              # 타이머에 의해 호출되어 계산 결과를 토픽으로 발행
        if self.keyboard_ctrl.robot_exit:                   # KeyboardController의 robot_exit 플래그를 확인합니다. True이면 아래 코드를 실행
            sys.exit(0)                                     # 프로그램을 정상 종료
            print("[INFO] END")

        wheel_speeds, steering_angles = self.keyboard_ctrl.compute_ackermann()      # 아크만 계산 호출
        
        speed_msg = Float64MultiArray(data=wheel_speeds)                            # Float64MultiArray 메시지 객체를 생성합니다. data에 wheel_speeds 리스트를 설정
        angle_msg = Float64MultiArray(data=steering_angles)                         # Float64MultiArray 메시지 객체를 생성합니다. data에 steering_angles 리스트를 설정
        self.speed_publisher.publish(speed_msg)                                     # speed_publisher를 통해 speed_msg를 /wheel_speeds 토픽으로 발행
        self.angle_publisher.publish(angle_msg)                                     # angle_publisher를 통해 angle_msg를 /steering_angles 토픽으로 발행

def main():                     # ROS2 노드를 초기화하고 실행
    rclpy.init()                # ROS2를 초기화합니다. (노드 실행 전에 필수, rclpy 라이브러리 준비.)
    node = KeyboardNode()       # KeyboardNode 클래스의 인스턴스를 생성하고 node 변수에 저장
    rclpy.spin(node)            # node를 스핀(무한 루프)하여 토픽/타이머/서비스 등을 처리
    rclpy.shutdown()            # ROS2를 종료합니다. (자원 해제, 프로그램 끝.)

if __name__ == "__main__":      # 이 파일이 직접 실행될 때(모듈로 import되지 않을 때) 아래 코드를 실행
    main()                      # main 함수를 호출
