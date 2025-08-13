import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import PortHandler, PacketHandler

# 포트 연결
PORT_CON           = '/dev/ttyACM1'                # 통신 포트 설정

# 다이나믹셀 설정관련
DXL_IDS             = [10, 11, 12, 13, 14, 15]     # 다이나믹셀 아이디

# slave 피드백 관련
FEEDBACK_CURRENT    = 1193                         # slave와 위치 차이 발생시 전류
DIFF_THRESHOLD      = 1000                           # 위치 차이 임계값 100 이상 차이시 
PERSISTENCE_COUNT   = 5                           # 지속 카운트: diff가 threshold 초과하는 루프 횟수 (부드러움 위해 추가)

# 컨트롤러 관련
POS_LIMITS = [[1000, 3000], [2000, 3800], [100, 2000], [1500, 3000], [0, 4095], [2000, 3000]]     # 위치 제한 값
NORMAL_CURRENTS     = [0, 10, 60, 10, 0, 0]     # 각 모터별 정상 동작시 전류 (필요에 따라 개별 값으로 변경)
STIFF_CURRENT       = 1193                           # 제한 범위 초과시 전류

# 부드러운 움직임
# PROFILE_VELOCITY    = 200
# PROFILE_ACCEL       = 100

class MasterNode(Node):
    def __init__(self):
        super().__init__('robot_arm_master')                                                                                    # ROS2 프레임워크에 노드가 올라감(상호 통신 등 가능)
        self.publisher_ = self.create_publisher(Int32MultiArray, 'master_positions', 10)                                        # master_positions 토픽으로 정수배열(모터 위치) 발행(per 0.1초)
        self.subscription = self.create_subscription(Int32MultiArray, 'slave_positions', self.slave_positions_callback, 10)     # slave_positions 토픽 구독(slave가 보내는 피드백 위치 복수개 수신)
        self.dxl_ids = DXL_IDS
        self.portHandler = PortHandler(PORT_CON)
        self.packetHandler = PacketHandler(2.0)

        if not self.portHandler.openPort():
            self.get_logger().error("포트를 열 수 없습니다.")
            raise RuntimeError("Failed to open the port")
        if not self.portHandler.setBaudRate(57600):
            self.get_logger().error("보드레이트 설정 실패.")
            raise RuntimeError("Failed to set baudrate")

        for i, dxl_id in enumerate(self.dxl_ids):
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 11, 5)  # 전류기반 위치모드
            # self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, PROFILE_VELOCITY)
            # self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 108, PROFILE_ACCEL)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 1)

            current_pos = self.read_position(dxl_id)
            self.write_goal_position(dxl_id, current_pos)           # 목표 위치를 현재 위치로 설정합니다(갑작스런 점프 방지).
            self.write_goal_current(dxl_id, NORMAL_CURRENTS[i])     # 각 모터별 목표 전류 NORMAL로 설정

        self.slave_positions = None                                 # 슬레이브 피드백 위치 보관공간 초기화
        self.diff_counters = [0] * len(self.dxl_ids)                # 각 모터별 diff 지속 카운터 초기화 (부드러움 위해 추가)
        self.timer = self.create_timer(0.01, self.control_loop)     
        self.get_logger().info("Master node initialized")

    def read_position(self, dxl_id):                                # 개별 모터의 현재 위치값 읽기
        pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
        if pos >= (1 << 31):
            pos -= (1 << 32)
        return pos

    def write_goal_position(self, dxl_id, pos):                     # 목표 위치
        if pos < 0:
            pos += (1 << 32)
        self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 116, pos)

    def write_goal_current(self, dxl_id, current):                  # 목표 전류
        if current < 0:
            current += (1 << 16)
        self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, 102, current)

    def slave_positions_callback(self, msg):                        # slave에서 보낸 위치값 배열을 콜백으로 받아 저장
        if len(msg.data) == len(self.dxl_ids):
            self.slave_positions = msg.data
        else:
            self.get_logger().warning("Received slave positions with mismatched motor count")

    def control_loop(self):
        positions = []                              # 마스터 모터 위치들을 저장할 리스트 생성.
        log_msg = ""                                # 로그 메시지 문자열 초기화.
        for i, dxl_id in enumerate(self.dxl_ids):   # 모든 모터에 대해 반복
            pos = self.read_position(dxl_id)        # 현재 모터 위치 읽기
            positions.append(pos)                   # positions 리스트에 현재 위치 추가.
            log_msg += f"ID{dxl_id} Pos: {pos}; "   # 로그 메세지에 현재 모터 위치를 추가.

            min_pos, max_pos = POS_LIMITS[i]        # 해당 모터의 허용 최소/최대 위치값 불러오기.
            goal_pos = pos                          # 기본 목표 위치를 현재 위치로 설정.
            goal_current = NORMAL_CURRENTS[i]       # 각 모터별 기본 목표 전류 설정

            if pos < min_pos:                       # 범위보다 작으면 목표 위치를 최소값으로, 전류를 강하게 설정.
                goal_pos = min_pos
                goal_current = STIFF_CURRENT
            elif pos > max_pos:                     # 범위보다 크면 목표 위치를 최대값으로, 전류를 강하게 설정.
                goal_pos = max_pos
                goal_current = STIFF_CURRENT

            if self.slave_positions is not None:                            # 슬레이브 위치 데이터가 있다면
                diff = abs(pos - self.slave_positions[i])                   # 마스터와 슬레이브 위치 차이를 계산
                if diff > DIFF_THRESHOLD:                                   
                    self.diff_counters[i] += 1                              # diff 초과 시 카운터 증가
                    if self.diff_counters[i] >= PERSISTENCE_COUNT:          # 지속 카운트 초과 시에만 전류 증가
                        goal_current = max(goal_current, FEEDBACK_CURRENT)
                else:
                    self.diff_counters[i] = 0                               # diff가 threshold 이하이면 카운터 리셋

            self.write_goal_position(dxl_id, goal_pos)      # 계산된 목표 위치를 모터에 전송
            self.write_goal_current(dxl_id, goal_current)   # 계산된 목표 전류를 모터에 전송

        msg = Int32MultiArray()                     # 전송할 ROS 메시지 객체 생성
        msg.data = positions                        # 메시지에 현재 master 위치 데이터 목록 저장
        self.publisher_.publish(msg)                # master_positions 토픽으로 발행
        self.get_logger().info(log_msg.strip('; ')) # 현재 모든 모터의 ID와 위치를 로그에 출력

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
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()