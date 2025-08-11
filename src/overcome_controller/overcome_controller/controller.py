import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import PortHandler, PacketHandler

# 포트 연결 관련
PORT_CON        = '/dev/ttyACM0'        # 통신 포트 설정

# 다이나믹셀 설정관련
DI_ID_LEFT      = 21                    # 왼쪽 바퀴 컨트롤러
DI_ID_RIGHT     = 22                    # 오른쪽 바퀴 컨트롤러
GOAL_CURRENT    = 10                    # 초기위치로 돌아올 전류값

# 컨트롤러 관련
DI_MIN_POS      = 1592                  # 최소 속도 위치
ST_MIN_POS      = 2174                  # 역회전 시작
MIN_ZERO_POS    = 2200                  # 0
FIRST_POS       = 2275                  # 초기 컨트롤러 위치
MAX_ZERO_POS    = 2300                  # 0
ST_MAX_POS      = 2376                  # 정회전 시작
DI_MAX_POS      = 2730                  # 최대 속도 위치

# 모터관련
VEL_MAX         = 330                   # 모터 최대 속도
VEL_MIN         = -330                  # 모터 최소 속도


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'controller', 10)
        self.dxl_ids = [DI_ID_LEFT, DI_ID_RIGHT]
        self.portHandler = PortHandler(PORT_CON)
        self.packetHandler = PacketHandler(2.0)

        if not self.portHandler.openPort():
            self.get_logger().error("포트를 열 수 없습니다.")
            raise RuntimeError("Failed to open the port")
        if not self.portHandler.setBaudRate(57600):
            self.get_logger().error("보드레이트 설정 실패.")
            raise RuntimeError("Failed to set baudrate")

        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 11, 5)
            self.packetHandler.write1ByteTxRx(self.portHandler, DI_ID_RIGHT, 10, 1)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 1)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 116, FIRST_POS)
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, 102, GOAL_CURRENT)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("[INFO] 다이나믹셀 위치 및 전류 제어 노드 시작")

    def read_position(self, dxl_id):
        position, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
        position = position if position < (1 << 31) else position - (1 << 32)
        return position

    def map_position_to_velocity(self, pos: int) -> int:
        if pos <= ST_MIN_POS:
            if pos <= DI_MIN_POS:
                return VEL_MIN
            return int(-1 - (ST_MIN_POS - pos) * ((VEL_MAX - 1) / (ST_MIN_POS - DI_MIN_POS)))
        elif MIN_ZERO_POS <= pos <= MAX_ZERO_POS:
            return 0
        elif pos >= ST_MAX_POS:
            if pos >= DI_MAX_POS:
                return VEL_MAX
            return int(1 + (pos - ST_MAX_POS) * ((VEL_MAX - 1) / (DI_MAX_POS - ST_MAX_POS)))
        return 0

    def control_loop(self):
        velocities = []
        log_msg = ""
        for dxl_id in self.dxl_ids:
            pos = self.read_position(dxl_id)
            velocity = self.map_position_to_velocity(pos)
            velocities.append(velocity)
            log_msg += f"ID{dxl_id} Pos: {pos}, Vel: {velocity}; "

        msg = Int32MultiArray()
        msg.data = velocities
        self.publisher_.publish(msg)
        self.get_logger().info(log_msg.strip('; '))

    def __del__(self):
        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, 64, 0)
        self.portHandler.closePort()

def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
