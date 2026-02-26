"""
이 코드는 ROS2 시스템을 통해 로봇의 앞바퀴 조향(방향 전환)을 정밀하게 담당하는 소프트웨어입니다.
앞서 보신 인휠 모터가 로봇의 '발'이라면, 이 코드는 로봇의 방향을 정하는 '핸들' 역할을 합니다.

가장 중요한 특징은 '부드러운 움직임'입니다. 
바퀴 각도를 한 번에 팍 꺾는 것이 아니라, 목표 각도까지 잘게 나누어 이동시키는 스무딩(Smoothing) 기술이 들어있습니다. 
덕분에 기계적인 충격 없이 실제 자동차처럼 매끄럽게 바퀴 방향을 바꿀 수 있습니다.

또한, 이 프로그램은 ROS2 네트워크를 통해 전달되는 왼쪽과 오른쪽 바퀴의 각도 신호를 0.01초마다 실시간으로 확인합니다. 
받은 각도 값은 다이나믹셀 서보 모터가 이해할 수 있는 정밀한 수치로 변환되어 즉시 전송됩니다.

마지막으로, 로봇을 멈출 때는 바퀴를 자동으로 정중앙(11자)으로 정렬하고 모터 힘을 빼는 안전 종료 기능이 포함되어 있어 하드웨어의 파손을 방지합니다.
요약하자면, "상위 시스템에서 받은 방향 명령을 계산하여 실제 바퀴를 부드럽고 안전하게 꺾어주는 지능형 조향 장치"라고 할 수 있습니다.
"""
import math
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String  # Float64MultiArray로 변경, Twist 제거
from dynamixel_sdk import PortHandler, PacketHandler

class SteeringMotorController:
    def __init__(self, port_name='/dev/ttyACM0', baud_rate=1000000, steering_ids=(0, 1)):
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(baud_rate)):
            print("포트를 열거나 보드레이트 설정에 실패했습니다.")
            raise Exception("포트 연결 실패")
        self.steering_ids = steering_ids
        self.INIT_POSITION = 2048
        for _id in self.steering_ids:
            self.packet.write1ByteTxRx(self.port, _id, 64, 0)
            self.packet.write1ByteTxRx(self.port, _id, 11, 3)
            self.packet.write1ByteTxRx(self.port, _id, 64, 1)
            self.packet.write4ByteTxRx(self.port, _id, 116, self.INIT_POSITION)
        self.ROBOT_WIDTH = 347
        self.ROBOT_LENGTH = 450
        self.front_wheelbase = self.ROBOT_LENGTH / 2
        self.rear_wheelbase = self.ROBOT_LENGTH / 2

        self.current_positions = [self.INIT_POSITION for _ in self.steering_ids]
        self.STEERING_SMOOTH_STEP = 5  

    def steering_to_position(self, angle):
        return self.INIT_POSITION + int((angle / 45.0) * 512)

    def apply_steering_angles(self, angles):  # 새 메서드: 받은 각도 배열을 부드럽게 적용 (update_steering 대체)
        if len(angles) != 2:
            print("[ERROR] Invalid angles length, using default [0.0, 0.0]")
            angles = [0.0, 0.0]
        
        target_angles = angles + [0.0, 0.0]  # 뒷바퀴 고정: [FL, FR, RL=0, RR=0]으로 확장 (4WS 확장 대비)
        
        target_positions = [self.steering_to_position(a) for a in target_angles[:2]]  # 앞바퀴 2개만 (뒷바퀴 무시 가정)
        diffs = [target_positions[i] - self.current_positions[i] for i in range(len(target_positions))]
        abs_diffs = [abs(d) for d in diffs]
        max_diff = max(abs_diffs) if abs_diffs else 0
        
        if max_diff == 0:
            for i, motor_id in enumerate(self.steering_ids):
                self.packet.write4ByteTxRx(self.port, motor_id, 116, int(self.current_positions[i]))
            return target_angles[:2]  # 앞 2개만 반환
        
        for i, motor_id in enumerate(self.steering_ids):
            diff = diffs[i]
            if abs(diff) < 1:
                new_position = target_positions[i]
            else:
                step = self.STEERING_SMOOTH_STEP * abs(diff) / max_diff
                new_position = self.current_positions[i] + (step if diff > 0 else -step)
            self.current_positions[i] = new_position
            self.packet.write4ByteTxRx(self.port, motor_id, 116, int(new_position))
        return target_angles[:2]  # 앞 2개만 반환

    def shutdown(self):
        for _id in self.steering_ids:
            self.packet.write4ByteTxRx(self.port, _id, 116, self.INIT_POSITION)
            self.packet.write1ByteTxRx(self.port, _id, 64, 0)
        self.port.closePort()


class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')
        self.subscription = self.create_subscription(Float64MultiArray, '/steering_angles', self.angle_callback, 10)  # 수정: /steering_angles, Float64MultiArray
        self.publisher = self.create_publisher(String, '/motor_status', 10)  # 상태 피드백
        self.steering_ctrl = SteeringMotorController()
        self.angles = [0.0, 0.0]  # 초기 각도 배열
        self.timer = self.create_timer(0.01, self.control_loop)

    def angle_callback(self, msg: Float64MultiArray):
        self.angles = msg.data  # [left, right] 추출
        if len(self.angles) != 2:
            print("[ERROR] Invalid angles length, resetting to [0.0, 0.0]")
            self.angles = [0.0, 0.0]

    def control_loop(self):
        applied_angles = self.steering_ctrl.apply_steering_angles(self.angles)  # 수정: 받은 각도 직접 적용
        status = f"조향: Left:{applied_angles[0]:.1f}°, Right:{applied_angles[1]:.1f}°"
        self.publisher.publish(String(data=status))
        sys.stdout.write('\r' + status)
        sys.stdout.flush()

    def destroy_node(self):
        self.steering_ctrl.shutdown()
        super().destroy_node()

def main():
    rclpy.init()
    node = SteeringNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
