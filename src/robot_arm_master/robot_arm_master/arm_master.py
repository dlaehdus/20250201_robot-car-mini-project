import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import PortHandler, PacketHandler

# 포트 연결
PORT_CON = '/dev/ttyACM1'  # 통신 포트 설정

# 다이나믹셀 설정관련
DXL_IDS = [10, 11, 12, 13, 14, 15]  # 다이나믹셀 아이디

# 컨트롤러 관련
POS_LIMITS = [[1000, 3000], [2000, 3800], [100, 2000], [1500, 3000], [0, 4095], [2000, 3000]]  # 위치 제한 값
CURRENT_LIMITS = [[-800, 800], [-800, 800], [-800, 800], [-800, 800], [-800, 800], [-200, 200]]  # 전류 제한 값 (예시: -800 ~ 800mA, 모터별 조정 가능)
NORMAL_CURRENTS = [0, 0, 0, 0, 0, 0]  # 각 모터별 정상 동작시 전류
STIFF_CURRENT = 1193  # 위치 제한 초과 시 전류 (fallback)

# 피드백 상수
FEEDBACK_SCALE = 0.8  # slave current 초과분을 master에 반영할 스케일 (0~1 조절)

# 부드러운 움직임 (Slave와 일관성을 위해 추가)
PROFILE_VELOCITY    = 32767                                                              # 모터의 프로파일 속도(움직임 속도)
PROFILE_ACCEL       = 30                                                                 # 모터의 프로파일 가속도(가속 속도)를 상수로 정의합니다

class MasterNode(Node):
    def __init__(self):
        super().__init__('robot_arm_master')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'master_positions', 10)
        self.subscription = self.create_subscription(Int32MultiArray, 'slave_positions', self.slave_positions_callback, 10)

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
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)  # 토크 비활성화
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 11, 5)  # 전류기반 위치모드
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, PROFILE_VELOCITY)  # 프로파일 속도 설정 (추가)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 108, PROFILE_ACCEL)     # 프로파일 가속도 설정 (추가)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 1)  # 토크 활성화

            current_pos = self.read_position(dxl_id)
            self.write_goal_position(dxl_id, current_pos)
            self.write_goal_current(dxl_id, NORMAL_CURRENTS[i])

        self.slave_positions = None
        self.slave_currents = None  # slave 현재 전류 저장
        self.timer = self.create_timer(0.05, self.control_loop)  # slave와 주기 맞춤
        self.get_logger().info("Master node initialized")

    def read_position(self, dxl_id):
        pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
        if pos >= (1 << 31):
            pos -= (1 << 32)
        return pos

    def write_goal_position(self, dxl_id, pos):
        if pos < 0:
            pos += (1 << 32)
        self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 116, pos)

    def write_goal_current(self, dxl_id, current):
        if current < 0:
            current += (1 << 16)
        self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, 102, current)

    def slave_positions_callback(self, msg):
        num_motors = len(self.dxl_ids)
        if len(msg.data) != num_motors * 2:  # positions(6) + currents(6) = 12 (slave 코드에 맞춤)
            self.get_logger().warning("Received slave data with mismatched length")
            return

        # 데이터 분리
        self.slave_positions = msg.data[:num_motors]  # 앞 6: 위치
        self.slave_currents = msg.data[num_motors:]  # 뒤 6: 현재 전류

    def control_loop(self):
        positions = []
        log_msg = ""
        for i, dxl_id in enumerate(self.dxl_ids):
            pos = self.read_position(dxl_id)
            positions.append(pos)
            log_msg += f"ID{dxl_id} Pos: {pos}; "

            min_pos, max_pos = POS_LIMITS[i]
            min_curr, max_curr = CURRENT_LIMITS[i]  # 전류 제한 값
            mid_curr = (min_curr + max_curr) / 2  # 중간 값 (nominal)
            goal_pos = pos
            goal_current = NORMAL_CURRENTS[i]

            # 위치 제한 체크 (기존)
            if pos < min_pos:
                goal_pos = min_pos
                goal_current = STIFF_CURRENT
            elif pos > max_pos:
                goal_pos = max_pos
                goal_current = STIFF_CURRENT

            # 슬레이브 전류 기반 proportional 피드백 (개선)
            if self.slave_currents is not None:
                slave_curr = self.slave_currents[i]
                if slave_curr < min_curr or slave_curr > max_curr:
                    excess = abs(slave_curr - mid_curr)  # 초과분 계산
                    goal_current = int(NORMAL_CURRENTS[i] + FEEDBACK_SCALE * excess)  # 비례 반영
                    goal_current = min(goal_current, STIFF_CURRENT)  # 상한 제한
                    self.get_logger().warning(f"Slave motor ID {self.dxl_ids[i]} current exceeded limits: {slave_curr} (limits: {min_curr}~{max_curr}) - Applying proportional feedback current {goal_current} to corresponding master motor")

            # 명령 전송
            self.write_goal_position(dxl_id, goal_pos)
            self.write_goal_current(dxl_id, goal_current)

        # 현재 마스터 포지션 발행
        msg = Int32MultiArray()
        msg.data = positions
        self.publisher_.publish(msg)
        self.get_logger().info(log_msg.strip('; '))

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