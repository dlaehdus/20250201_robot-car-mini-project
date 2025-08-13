import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import PortHandler, PacketHandler

# 포트 연결
SLAVE_PORT_CON = '/dev/ttyACM0'                                                         # 통신 포트 설정

# 다이나믹셀 설정 관련
SLAVE_DXL_IDS = [20, 21, 22, 23, 24, 25]                                                # 다이나믹셀 아이디

# slave관련
SLAVE_POS_LIMITS = [[0, 4095], [0, 4095], [0, 4095], [0, 4095], [0, 4095], [0, 4095]]   # 제한 범위
SLAVE_MAX_CURRENTS = [1193, 1193, 1193, 1193, 1193, 1193]                               # 각 모터별 max 전류

# 부드러운 움직임
PROFILE_VELOCITY    = 32767                                                             # 모터의 프로파일 속도(움직임 속도)
PROFILE_ACCEL       = 40                                                                # 모터의 프로파일 가속도(가속 속도)를 상수로 정의합니다

class SlaveNode(Node):
    def __init__(self):
        super().__init__('robot_arm_slave')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'slave_positions', 10)
        self.subscription = self.create_subscription(Int32MultiArray, 'master_positions', self.master_positions_callback, 10)
        self.dxl_ids = SLAVE_DXL_IDS
        self.portHandler = PortHandler(SLAVE_PORT_CON)
        self.packetHandler = PacketHandler(2.0)

        if not self.portHandler.openPort():
            self.get_logger().error("포트를 열 수 없습니다.")
            raise RuntimeError("Failed to open the port")
        if not self.portHandler.setBaudRate(57600):
            self.get_logger().error("보드레이트 설정 실패.")
            raise RuntimeError("Failed to set baudrate")
        
        for i, dxl_id in enumerate(self.dxl_ids):
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 11, 5)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, PROFILE_VELOCITY)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 108, PROFILE_ACCEL)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 1)

            current_pos = self.read_position(dxl_id)
            self.write_goal_position(dxl_id, current_pos)                               # 목표 위치를 현재 위치로 설정 안하면 손을 놓자마자 힘없이 떨어짐
            self.write_goal_current(dxl_id, SLAVE_MAX_CURRENTS[i])                      # 각 모터별 목표 전류 설정

        self.max_currents = SLAVE_MAX_CURRENTS                                          # max 전류 배열 저장 (발행용)
        self.timer = self.create_timer(0.01, self.control_loop)                         # 0.01초(100Hz)마다 control_loop 메서드를 호출하는 타이머를 생성
        self.get_logger().info("Slave node initialized")

    def read_position(self, dxl_id):                                # 개별 모터의 현재 위치값 읽기
        pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
        if pos >= (1 << 31):
            pos -= (1 << 32)
        return pos

    def read_current(self, dxl_id):                                 # 개별 모터의 현재 전류값 읽기
        current, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, 126)
        if current >= (1 << 15):
            current -= (1 << 16)
        return current

    def write_goal_position(self, dxl_id, pos):                     # 목표 위치
        if pos < 0:
            pos += (1 << 32)
        self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 116, pos)

    def write_goal_current(self, dxl_id, current):                  # 목표 전류
        if current < 0:
            current += (1 << 16)
        self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, 102, current)

    def master_positions_callback(self, msg):                       # 마스터로부터 위치 데이터를 받았을 때 호출되는 콜백 메서드
        if len(msg.data) != len(self.dxl_ids):                      # 받은 데이터 길이가 모터 수와 맞지 않으면 경고를 출력하고 종
            self.get_logger().warning("Received master positions with mismatched motor count")
            return
        for i, goal_pos in enumerate(msg.data):                             # 받은 위치 데이터를 반복하며 처리합니다.
            min_pos, max_pos = SLAVE_POS_LIMITS[i]                          # 해당 모터의 위치 제한을 가져옵니다.
            goal_pos = max(min_pos, min(goal_pos, max_pos))                 # 목표 위치를 제한 범위 내로 클램핑합니다.
            self.write_goal_position(self.dxl_ids[i], goal_pos)             # 클램핑된 목표 위치를 씁니다
            self.write_goal_current(self.dxl_ids[i], self.max_currents[i])  # 해당 모터의 최대 전류를 목표로 씁니다

    def control_loop(self):                             # 주기적으로 호출되는 제어 루프 메서드
        positions = []                                  # 위치 리스트를 초기화합니다.
        currents = []                                   # 현재 전류 리스트를 초기화합니다.
        log_msg = ""                                    # 로그 메시지 문자열을 초기화합니다.
        for i, dxl_id in enumerate(self.dxl_ids):       # 각 모터에 대해 반복합니다.
            pos = self.read_position(dxl_id)            # 위치를 읽습니다.
            current = self.read_current(dxl_id)         # 현재 전류를 읽습니다.
            positions.append(pos)                       # 리스트에 추가합니다.
            currents.append(current)                    # 리스트에 추가합니다.
            log_msg += f"ID{dxl_id} Pos: {pos} Present Current: {current}; "       # 로그에 모터 ID, 위치, 현재 전류, 목표 전류를 추가합니다.
        msg = Int32MultiArray()                                 # 발행할 메시지 객체를 생성합니다.
        msg.data = positions + currents + self.max_currents     # 데이터를 연결합니다: 위치 배열 + 현재 전류 배열 + 목표 전류 배열.
        self.publisher_.publish(msg)                            # 'slave_positions' 토픽으로 발행합니다. 마스터가 이 데이터를 받습니다.
        self.get_logger().info(log_msg.strip('; '))             # 로그를 출력합니다. 마지막 ';'를 제거합니다.

    def destroy_node(self):
        for dxl_id in self.dxl_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
            if dxl_comm_result != 0:
                self.get_logger().error(f"{self.packetHandler.getTxRxResult(dxl_comm_result)} for torque off ID {dxl_id}")
            if dxl_error != 0:
                self.get_logger().error(f"{self.packetHandler.getRxPacketError(dxl_error)} for torque off ID {dxl_id}")
        self.portHandler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SlaveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()