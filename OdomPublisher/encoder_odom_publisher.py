import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.1)

        self.last_count = 0
        self.last_timestamp = time.time()

        # エンコーダの仕様とタイヤのサイズに基づいたパラメータ
        self.encoder_resolution = 600  # エンコーダの分解能（1回転あたりのパルス数）
        self.wheel_diameter = 0.1  # タイヤの直径 (メートル)
        self.wheel_circumference = 3.141592653589793 * self.wheel_diameter  # タイヤの円周

        self.timer = self.create_timer(0.1, self.send_and_receive)

    def send_and_receive(self):
        self.serial_port.write(b'c')
        
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                self.process_data(line)

    def process_data(self, data):
        try:
            current_count = int(data)
            current_timestamp = time.time()
            
            delta_count = current_count - self.last_count
            delta_time = current_timestamp - self.last_timestamp
            
            if delta_time > 0:  # ゼロ除算防止
                # タイヤが1回転するのに必要なカウント数で割って回転数を求め、
                # タイヤの円周で移動距離を計算し、時間で割って速度を算出
                rotations = delta_count / self.encoder_resolution
                distance = rotations * self.wheel_circumference  # 移動距離
                velocity = distance / delta_time  # 速度

                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                odom_msg.twist.twist.linear.x = velocity
                
                self.odom_pub.publish(odom_msg)

                self.get_logger().info(f'Published Odometry: velocity={velocity} m/s')
            
            self.last_count = current_count
            self.last_timestamp = current_timestamp
        
        except ValueError as e:
            self.get_logger().error(f'Received non-integer data: {e}')

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.serial_port.close()
        odom_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
