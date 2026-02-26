"""
이 코드는 ROS2 시스템을 통해 4개의 인휠 모터(In-Wheel Motor)를 실시간으로 제어하는 로봇의 구동 엔진입니다.

먼저, 프로그램은 우리가 흔히 사용하는 속도 숫자를 모터 하드웨어가 이해할 수 있는 16진수 바이트(Byte) 형태의 데이터로 변환합니다.
이때 데이터가 전송 중에 깨지지 않도록 CRC8이라는 특수 검증 코드를 붙여 통신의 안전성을 확보합니다.

또한, 이 코드는 로봇의 신경망인 ROS2 네트워크와 긴밀하게 연결되어 있습니다.
상위 제어기가 "어느 정도 속도로 가라"는 신호를 보내면 이를 즉시 수신(구독)하고,
0.01초라는 매우 짧은 간격으로 모터에 명령을 전달합니다. 덕분에 로봇은 지연 시간 없이 부드럽고 민첩하게 움직일 수 있습니다.

마지막으로, 바퀴가 서로 반대 방향으로 장착된 물리적 구조를 고려하여 회전 방향을 자동으로 보정해 주는 지능적인 기능도 포함하고 있습니다.
한마디로 이 코드는 상위의 주행 명령을 하드웨어의 실제 움직임으로 연결해 주는 가장 밑바단의 핵심적인 가교 역할을 수행하는 소프트웨어입니다.
"""


import serial                                           # serial 모듈을 불러옵니다. 시리얼 통신(USB 포트 등을 통해 하드웨어와 데이터 주고받기)을 위한 라이브러리로, 인휠 모터와의 연결에 사용
import struct                                           # struct 모듈을 불러옵니다. 바이트 데이터 패킹/언패킹(예: 정수를 hex 바이트로 변환)에 사용
import sys                                              # sys 모듈을 불러옵니다. 시스템 관련 기능(예: 표준 출력 stdout, 프로그램 종료 sys.exit)을 사용하기 위함입니다. 로그 출력이나 종료 처리에 활용
import rclpy                                            # rclpy 모듈을 불러옵니다. ROS2의 Python 인터페이스로, 노드 생성, 스핀, 초기화 등의 기본 기능을 제공
from rclpy.node import Node                             # rclpy.node 모듈에서 Node 클래스를 불러옵니다. ROS2 노드를 만들기 위한 기본 클래스입니다. InWheelNode가 이 클래스를 상속받아 사용
from std_msgs.msg import Float64MultiArray, String      # std_msgs.msg 모듈에서 Float64MultiArray와 String 메시지 타입을 불러옵니다

class InWheelMotorController:
    def __init__(self, port_list=("/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3", "/dev/ttyACM4"),
                 velocity_ids=(0x01, 0x02, 0x03, 0x04)):        # velocity_ids를 16진수 튜플(모터 ID)로 기본값 설정
        self.port_list = port_list                              # 입력된 포트 리스트를 클래스 변수에 저장
        self.velocity_ids = velocity_ids                        # 입력된 모터 ID 리스트를 클래스 변수에 저장
        self.serial_connections = []                            # 시리얼 연결 리스트를 빈 리스트로 초기화합니다. (각 포트의 연결 객체를 저장할 곳.)
        for port in self.port_list:                             # 포트 리스트를 반복합니다. 각 포트에 대해 아래 코드를 실행
            ser = self.connect_serial(port)                     # connect_serial 메서드를 호출하여 해당 포트로 시리얼 객체를 생성
            self.serial_connections.append(ser)                 # 생성된 시리얼 객체를 리스트에 추가합니다. (4개 포트 → 4개 연결.)

    def connect_serial(self, port, baudrate=115200, timeout=1):
        try:                                    # 블록 시작: 예외 처리를 위한 블록. 연결 시도 중 에러를 잡습니다.
            ser = serial.Serial(                # serial.Serial 객체를 생성합니다. 시리얼 연결 설정 시작.
                port=port,                      # port 파라미터: 연결할 포트 경로 설정
                baudrate=baudrate,              # baudrate 파라미터: 통신 속도 설정
                parity=serial.PARITY_NONE,      # parity: 패리티 검사 없음
                stopbits=serial.STOPBITS_ONE,   # stopbits: 스톱 비트 1개
                bytesize=serial.EIGHTBITS,      # bytesize: 8비트 데이터
                timeout=timeout                 # timeout: 읽기 대기 시간 설정.
            )
            print(f"[INFO] Connected to {port}")
            return ser
        except serial.SerialException as e:
            print(f"[ERROR] Failed to connect to {port}: {e}")
            raise
        
    @staticmethod
    def decimal_to_hex_bytes(decimal):
        hex_string = format(struct.unpack('>H', struct.pack('>h', int(decimal)))[0], 'x').zfill(4)
        return int(hex_string[:2], 16), int(hex_string[2:], 16)

    @staticmethod
    def calculate_crc(data):
        if not isinstance(data, bytes):                             # 조건 검사: data가 bytes 타입인지 확인합니다. 아니면 아래 코드를 실행합니다
            data = bytes(data)                                      # data를 bytes 타입으로 변환합니다. (리스트나 다른 타입을 바이트로 만들어 CRC 계산 준비.)
        CRC8_MAXIM_table = (
            0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
            0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
            0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
            0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
            0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
            0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
            0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
            0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
            0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
            0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
            0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
            0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
            0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
            0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
            0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
            0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
            0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
            0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
            0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
            0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
            0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
            0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
            0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
            0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
            0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
            0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
            0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
            0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
            0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
            0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
            0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
            0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
        )
        crc = 0x00                              # CRC 초기값을 0x00으로 설정합니다. (계산 시작점.)
        for byte in data:                       # data의 각 바이트를 반복합니다. (바이트 단위로 CRC 업데이트.)
            index = (crc ^ int(byte)) & 0xFF    # index 계산: 현재 crc와 byte의 XOR(^) 연산, & 0xFF로 0~255 범위 마스킹합니다. (테이블 인덱스 생성.)
            crc = CRC8_MAXIM_table[index]       # 테이블에서 새로운 crc 값을 가져와 업데이트합니다.
        return crc

    @staticmethod
    def send_data(ser, data):
        ser.write(data)

    def set_velocity(self, ser, ID, speed):
        speed_H, speed_L = self.decimal_to_hex_bytes(speed)
        data_temp = bytes([int(ID) & 0xFF, 0x64, int(speed_H) & 0xFF, int(speed_L) & 0xFF, 0, 0, 0, 0, 0])
        crc = self.calculate_crc(data_temp)
        data = data_temp + bytes([crc])
        self.send_data(ser, data)
        print(f"[INFO] set_velocity for ID {ID}: speed {speed}")

    def brake(self, ser, ID):
        data_temp = bytes([int(ID) & 0xFF, 0x64, 0, 0, 0, 0, 0, 0xFF, 0])
        crc = self.calculate_crc(data_temp)
        data = data_temp + bytes([crc])
        self.send_data(ser, data)
        print(f"[INFO] brake for ID {ID}")

    def set_velocities_individual(self, speeds, signs=None):  # 수정: signs 파라미터 추가 (기본 None)
        if len(speeds) != 4:
            print("[ERROR] Speeds list must have 4 values")
            return
        # signs 기본값 설정 (기본: [-1, 1, -1, 1] – 왼쪽 반대, 오른쪽 정방향)
        if signs is None:
            signs = [-1, 1, -1, 1]
        elif len(signs) != 4:
            print("[ERROR] Signs list must have 4 values")
            return
        # 각 모터에 개별 속도와 방향 적용
        for i, (ser, motor_id, sign) in enumerate(zip(self.serial_connections, self.velocity_ids, signs)):
            target_velocity = sign * speeds[i]  # 속도에 방향(부호) 곱함
            self.set_velocity(ser, motor_id, target_velocity)
            print(f"[INFO] Set motor {i+1} (ID: {motor_id}): speed {speeds[i]}, sign {sign}, target {target_velocity}")
    
    def shutdown(self):                         # 모든 모터를 안전하게 정지시키고 연결을 끊습니다
        for ser, motor_id in zip(self.serial_connections, self.velocity_ids):
            self.brake(ser, motor_id)
            ser.close()

class InWheelNode(Node):
    def __init__(self):
        super().__init__('inwheel_node')                                                                            # 상위 클래스(Node)의 init 메서드를 호출합니다. 노드 이름을 'inwheel_node'로 설정합니다. (ROS2에서 노드를 등록합니다.)
        self.subscription = self.create_subscription(Float64MultiArray, '/wheel_speeds', self.speed_callback, 10)   # /wheel_speeds 토픽에서 Float64MultiArray 메시지를 구독할 subscription을 생성합니다. 콜백 함수는 self.speed_callback, queue_size=10.
        self.publisher = self.create_publisher(String, '/motor_status', 10)                                         # /motor_status 토픽으로 String 메시지를 발행할 publisher를 생성합니다. queue_size=10. (모터 상태 문자열 발행용.)
        self.inwheel_ctrl = InWheelMotorController()                                                                # InWheelMotorController 클래스의 인스턴스를 생성하고 self.inwheel_ctrl 변수에 저장합니다. (모터 컨트롤러 객체 생성.)
        self.speeds = [0.0, 0.0, 0.0, 0.0]                                                                          # speeds 변수를 4개 0.0으로 초기화된 리스트로 설정
        self.timer = self.create_timer(0.01, self.control_loop)                                                     # 0.01초 간격으로 self.control_loop 메서드를 호출하는 타이머를 생성

    def speed_callback(self, msg: Float64MultiArray):
        self.speeds = msg.data                                      # msg.data를 self.speeds에 저장합니다
        if len(self.speeds) != 4:                                   # 조건 검사: speeds 길이가 4가 아니면 아래 코드를 실행합니다
            print("[ERROR] Invalid speeds length")                  # 오류 메시지 출력
            self.speeds = [0.0, 0.0, 0.0, 0.0]                      # speeds를 초기값으로 재설정합니다. (잘못된 데이터 무시.)

    def control_loop(self):                                                                                     # 타이머에 의해 주기적으로 호출되어 모터 제어합니다.
        self.inwheel_ctrl.set_velocities_individual(self.speeds)                                                # inwheel_ctrl의 set_velocities_individual 메서드를 호출하여 self.speeds를 적용
        status = f"속도: FL:{self.speeds[0]}, FR:{self.speeds[1]}, RL:{self.speeds[2]}, RR:{self.speeds[3]}"     # status 문자열 생성: 4개 바퀴 속도를 포맷팅
        self.publisher.publish(String(data=status))                                                             # publisher를 통해 status를 String 메시지로 /motor_status 토픽에 발행
        sys.stdout.write('\r' + status)                                                                         # 콘솔에 status를 출력합니다. '\r'로 이전 줄을 덮어쓰기(실시간 업데이트 효과).
        sys.stdout.flush()                                                                                      # stdout 버퍼를 플러시하여 즉시 출력

    def destroy_node(self):             # destroy_node 메서드를 정의합니다. 노드 종료 시 호출되어 자원 해제합니다.
        self.inwheel_ctrl.shutdown()    # inwheel_ctrl의 shutdown 메서드를 호출하여 모터 정지와 연결 종료
        super().destroy_node()          # 상위 클래스(Node)의 destroy_node 메서드를 호출하여 ROS2 노드 자원 해제

def main():
    rclpy.init()
    node = InWheelNode()        # InWheelNode 클래스의 인스턴스를 생성하고 node 변수에 저장
    try:
        rclpy.spin(node)        # node를 스핀(무한 루프)하여 토픽/타이머 등을 처리
    finally:                    # finally 블록: try 종료 후 항상 실행
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
