import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct

# 포트 연결 관련
PORT_CON_1 = "/dev/ttyACM5"
PORT_CON_2 = "/dev/ttyACM2"

class InWheelMotorController:
    def __init__(self,port_list=(PORT_CON_1, PORT_CON_2),velocity_ids=(0x01, 0x01),reverse_index=0,baudrate=115200):
        self.port_list = port_list
        self.velocity_ids = velocity_ids
        self.reverse_index = reverse_index
        self.serial_connections = []
        for port in self.port_list:
            ser = self.connect_serial(port, baudrate)
            self.serial_connections.append(ser)

    def connect_serial(self, port, baudrate):
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            print(f"[INFO] Connected to {port}")
            return ser
        except serial.SerialException as e:
            print(f"[ERROR] Failed to connect {port}: {e}")
            raise

    @staticmethod
    def decimal_to_bytes(value: int) -> bytes:
        return struct.pack('>h', int(value))

    @staticmethod
    def calculate_crc(data: bytes) -> int:
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
        crc = 0x00
        for b in data:
            crc = CRC8_MAXIM_table[crc ^ b]
        return crc

    def send_velocity_command(self, ser, motor_id, speed):
        speed_bytes = self.decimal_to_bytes(speed)
        packet_no_crc = bytes([motor_id,0x64,speed_bytes[0],speed_bytes[1],0, 0, 0, 0, 0])
        crc = self.calculate_crc(packet_no_crc)
        packet = packet_no_crc + bytes([crc])
        ser.write(packet)

    def send_speeds(self, speeds):
        for i, (ser, motor_id) in enumerate(zip(self.serial_connections, self.velocity_ids)):
            speed = speeds[i]
            if i == self.reverse_index:
                speed = -speed
            self.send_velocity_command(ser, motor_id, speed)

    def shutdown(self):
        for ser, motor_id in zip(self.serial_connections, self.velocity_ids):
            self.send_velocity_command(ser, motor_id, 0)  # 모터 정지
            ser.close()


class InWheelMotorNode(Node):
    def __init__(self,
                 port_list=(PORT_CON_1, PORT_CON_2),
                 velocity_ids=(0x01, 0x01),
                 reverse_index=0):
        super().__init__('inwheel_motor_node')
        self.controller = InWheelMotorController(port_list, velocity_ids, reverse_index)
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'controller',
            self.speed_callback,
            10
        )
        self.get_logger().info("[INFO] InWheelMotorNode started and subscribed to 'controller'")

    def speed_callback(self, msg: Int32MultiArray):
        if len(msg.data) < len(self.controller.velocity_ids):
            self.get_logger().warn("[WARN] 입력 속도 데이터 길이가 포트 수와 일치하지 않습니다.")
            return
        speeds = msg.data[:len(self.controller.velocity_ids)]
        self.controller.send_speeds(speeds)
        self.get_logger().info(f"[INFO] Speeds sent: {speeds}")

    def destroy_node(self):
        self.controller.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InWheelMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
