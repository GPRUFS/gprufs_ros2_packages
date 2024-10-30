import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import socket
import struct

class ROS2NodeUDP(Node):
    def __init__(self):
        super().__init__('matlab_udp_link')
        
        # Configuração do publisher para publicar no tópico cmd_vel
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        
        # Configuração do subscriber para o tópico scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/lidar',
            self.scan_callback,
            10
        )
        
        # Configuração de sockets UDP
        self.udp_ip = "127.0.0.1"
        self.udp_port_receive = 12345  # Porta para receber comandos de velocidade
        self.udp_port_send = 12346     # Porta para enviar dados de Lidar
        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_receive.bind((self.udp_ip, self.udp_port_receive))
        
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Inicializar temporizador para verificar recepção UDP periodicamente
        self.create_timer(0.03, self.receive_udp_data)
        
    def receive_udp_data(self):
        # Recebe dados de velocidade via UDP e publica no tópico cmd_vel.
        self.sock_receive.settimeout(0.01)
        try:
            data, _ = self.sock_receive.recvfrom(1024)
            vL, vR = struct.unpack('ff', data)
            self.get_logger().info(f'Received UDP Twist: vL={vL}, vR={vR}')
            
            # Publica o comando de velocidade recebido via UDP
            twist_msg = Twist()
            twist_msg.angular.x = vL
            twist_msg.angular.y = vR
            self.publisher.publish(twist_msg)
            
        except socket.timeout:
            pass  # Sem dados para receber

    def scan_callback(self, msg):
        # Callback para enviar dados de Lidar via UDP.
        # Envia todo o vetor de ranges para o MATLAB/Octave via UDP
        ranges = msg.ranges
        data = struct.pack('f' * len(ranges), *ranges)  # Empacota todos os valores do vetor

        # Envia os dados via UDP
        self.sock_send.sendto(data, (self.udp_ip, self.udp_port_send))
        
        self.get_logger().info(f'Sent Lidar ranges via UDP...')

def main(args=None):
    rclpy.init(args=args)
    node = ROS2NodeUDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
