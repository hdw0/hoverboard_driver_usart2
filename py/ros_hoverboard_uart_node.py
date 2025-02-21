import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import Odometry
import serial
import threading
import time
import math

# Настройки последовательного порта
SERIAL_PORT = '/dev/ttyUSB0'  # измените при необходимости
BAUD_RATE = 115200

class HoverboardUARTNode(Node):
    def __init__(self):
        super().__init__('hoverboard_uart_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        except Exception as e:
            self.get_logger().error(f'Не удалось открыть порт {SERIAL_PORT}: {e}')
            return
        
        self.reader_thread = threading.Thread(target=self.serial_reader)
        self.reader_thread.daemon = True
        self.reader_thread.start()
    
    def serial_reader(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("O"):
                    parts = line.split()
                    if len(parts) == 4:
                        _, x_str, y_str, theta_str = parts
                        try:
                            x = float(x_str)
                            y = float(y_str)
                            theta = float(theta_str)
                            self.publish_odometry(x, y, theta)
                        except ValueError:
                            self.get_logger().warn(f'Неверный формат одометрии: {line}')
            except Exception as e:
                self.get_logger().error(f'Ошибка чтения из serial: {e}')
                time.sleep(0.1)
    
    def publish_odometry(self, x, y, theta):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position = Point(x, y, 0.0)
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        odom.pose.pose.orientation = Quaternion(0.0, 0.0, qz, qw)
        self.odom_pub.publish(odom)
    
    def cmd_vel_callback(self, msg):
        max_pwm = 1000
        v = msg.linear.x
        w = msg.angular.z
        left = int((v - w) * max_pwm)
        right = int((v + w) * max_pwm)
        left = max(min(left, max_pwm), -max_pwm)
        right = max(min(right, max_pwm), -max_pwm)
        command_str = f"M {left} {right}\n"
        self.get_logger().info(f'Отправка команды: {command_str.strip()}')
        self.ser.write(command_str.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = HoverboardUARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

