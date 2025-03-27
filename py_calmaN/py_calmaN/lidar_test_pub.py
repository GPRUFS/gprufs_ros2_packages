import rclpy
import serial 
import cv2
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from pyydlidar import YDLidarX2
import math


class WalkPublisher(Node):

    def __init__(self):
        super().__init__('walk_publisher')
        self.publisher_lidar = self.create_publisher(LaserScan, '/robot/lidar', 1)
        
        # Configuração do YDLidar X2
        self.lidar = YDLidarX2()
        port = "COM6"  # Ajuste para sua porta serial
        if not self.lidar.connect(port):
            print(f"Cannot connect to YDLidar X2 on {port}")
            exit(1)
        
        # Configurações do lidar X2
        self.lidar.set_scan_frequency(10.0)  # 10 Hz
        self.lidar.set_rotation_frequency(10.0)  # 10 Hz
        
        # Timer para leitura do lidar
        timer_period = 3.333 * 1e-3  # seconds (300 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Configuração da visualização
        plt.ion()  
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X (metros)')
        self.ax.set_ylabel('Y (metros)')
        self.ax.set_title('Leituras do LIDAR X2')
        self.ax.axis('equal')
        self.ax.grid(True)

    def timer_callback(self):
        self.sendMeasures()

    def sendMeasures(self):
        scan_data = self.lidar.get_scan_data()
        if scan_data:
            lidar_msg = self.convert_scan_to_msg(scan_data)
            self.publisher_lidar.publish(lidar_msg)
            self.visualize_scan(scan_data)

    def convert_scan_to_msg(self, scan_data):
        msg = LaserScan()
        
        # Configura os parâmetros da mensagem LaserScan
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        
        # Configura os parâmetros do scan para o X2
        msg.angle_min = math.radians(scan_data.config.min_angle)
        msg.angle_max = math.radians(scan_data.config.max_angle)
        msg.angle_increment = math.radians(scan_data.config.angle_step)
        msg.time_increment = 1.0 / scan_data.config.scan_frequency / len(scan_data.ranges)
        msg.scan_time = 1.0 / scan_data.config.scan_frequency
        msg.range_min = scan_data.config.min_range
        msg.range_max = scan_data.config.max_range
        
        # Preenche as leituras de distância
        msg.ranges = scan_data.ranges
        
        return msg
    
    def visualize_scan(self, scan_data):
        # Converte as leituras polares para coordenadas cartesianas
        angles = np.linspace(scan_data.config.min_angle, scan_data.config.max_angle, len(scan_data.ranges))
        angles_rad = np.radians(angles)
        
        # Ajuste de orientação (π/2 para alinhar com a frente do robô)
        angles_rad_adjusted = angles_rad - (np.pi/2.0)
        
        x = scan_data.ranges * np.cos(angles_rad_adjusted)
        y = scan_data.ranges * np.sin(angles_rad_adjusted)
        
        # Plotagem
        self.ax.clear()
        self.ax.plot(x, y, 'bo', markersize=2)
        self.ax.set_xlabel('X (metros)')
        self.ax.set_ylabel('Y (metros)')
        self.ax.set_title('Leituras do LIDAR X2')
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