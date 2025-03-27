import rclpy
import serial 
import cv2
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from ydlidar import YDLidar
from cv_bridge import CvBridge
import math


class WalkPublisher(Node):

    def __init__(self):
        super().__init__('walk_publisher')
        self.publisher_lidar = self.create_publisher(LaserScan, '/robot/lidar', 1)
        
        # Configuração do YDLidar
        self.lidar = YDLidar()
        port = "COM6"  # Ajuste para sua porta serial
        if not self.lidar.connect(port):
            print(f"Cannot connect to YDLidar on {port}")
            exit(1)
        
        # Configurações do lidar (ajuste conforme seu modelo específico)
        scan_frequency = 10.0  # Hz
        self.lidar.set_scan_frequency(scan_frequency)
        
        # Timer para leitura do lidar
        timer_period = 3.333 * 1e-3  # seconds (300 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Configuração da visualização
        plt.ion()  
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X (metros)')
        self.ax.set_ylabel('Y (metros)')
        self.ax.set_title('Leituras do LIDAR')
        self.ax.axis('equal')
        self.ax.grid(True)

    def timer_callback(self):
        self.sendMeasures()

    def sendMeasures(self):
        scan = self.lidar.get_lidar_data()
        if scan:
            lidar_msg = self.convert_scan_to_msg(scan)
            self.publisher_lidar.publish(lidar_msg)
            self.visualize_scan(scan)

    def convert_scan_to_msg(self, scan):
        msg = LaserScan()
        
        # Configura os parâmetros da mensagem LaserScan
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        
        # Configura os parâmetros do scan (ajuste conforme seu lidar)
        msg.angle_min = math.radians(scan.config.min_angle)
        msg.angle_max = math.radians(scan.config.max_angle)
        msg.angle_increment = math.radians(scan.config.angle_increment)
        msg.time_increment = 1.0 / scan.config.scan_frequency / len(scan.ranges)
        msg.scan_time = 1.0 / scan.config.scan_frequency
        msg.range_min = scan.config.min_range
        msg.range_max = scan.config.max_range
        
        # Preenche as leituras de distância
        msg.ranges = scan.ranges
        
        return msg
    
    def visualize_scan(self, scan):
        # Converte as leituras polares para coordenadas cartesianas
        angles = np.linspace(scan.config.min_angle, scan.config.max_angle, len(scan.ranges))
        angles_rad = np.radians(angles)
        
        # Ajuste de orientação (se necessário)
        angles_rad_adjusted = angles_rad - (np.pi/2.0)
        
        x = scan.ranges * np.cos(angles_rad_adjusted)
        y = scan.ranges * np.sin(angles_rad_adjusted)
        
        # Plotagem
        self.ax.clear()
        self.ax.plot(x, y, 'bo', markersize=2)
        self.ax.set_xlabel('X (metros)')
        self.ax.set_ylabel('Y (metros)')
        self.ax.set_title('Leituras do LIDAR')
        self.ax.axis('equal')
        self.ax.grid(True)
        
        # Atualiza o gráfico
        plt.pause(0.001)
        self.fig.canvas.draw_idle()

    def __del__(self):
        if hasattr(self, 'lidar') and self.lidar:
            self.lidar.disconnect()


def main(args=None):
    # Inicialização
    rclpy.init(args=args)

    # Criação do nó
    walk_publisher = WalkPublisher()

    # Execução principal
    try:
        rclpy.spin(walk_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        walk_publisher.destroy_node()
        rclpy.shutdown()
        print("Execução encerrada pelo usuário")


if __name__ == '__main__':
    main()