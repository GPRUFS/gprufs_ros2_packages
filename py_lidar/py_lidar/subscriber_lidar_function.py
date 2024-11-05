import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class lidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription_ = self.create_subscription(LaserScan, '/robot/lidar', self.lidar_callback, 1)
        self.subscription_ # para previnir o "warning" de variável sem uso
        self.lidar_msg = LaserScan()
        self.last_time = time.time()
        self.fps = 0.0
        self.fps_file = open("D:/fps_data.txt", "w")  # Abre o arquivo para escrita

    def lidar_callback(self, msg):
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        self.last_time = current_time

        if elapsed_time > 0:
            self.fps = 1.0 / elapsed_time

        # Escreve o valor de FPS no arquivo
        self.fps_file.write(f"{self.fps:.2f}\n")

        self.lidar_msg = msg
        self.get_logger().info(f'Dados Recebidos - FPS: {self.fps:.2f}')

    def destroy(self):
        self.fps_file.close()  # Fecha o arquivo ao destruir o nó
        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    lidar_subscriber = lidarSubscriber()

    # faz o spin do nó para habilitar recebimento de callbacks
    rclpy.spin(lidar_subscriber)

    # especificação de shutdown
    lidar_subscriber.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()