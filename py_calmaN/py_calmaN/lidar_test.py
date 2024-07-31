import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarTest(Node):

    def __init__(self):
        super().__init__('lidar_test')
        self.subscription_ = self.create_subscription(LaserScan, '/robot/lidar', self.lidar_callback, 1)
        self.subscription_ # para previnir o "warning" de variável sem uso
        plt.ion()  # Ativar modo interativo
        self.fig, self.ax = plt.subplots()


    def lidar_callback(self, msg):
        distances = np.array(msg.ranges)
        idc = (np.where(distances > 0))
        angles = np.linspace(msg.angle_min, msg.angle_max, len(distances))
        angles = angles[idc]
        distances = distances[idc]

        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        #indices = np.where(np.sin(angles) > 0)
        
        self.ax.clear()  # Limpar o gráfico anterior
        self.ax.plot(x, y, 'bo')
        #self.ax.plot(x[indices], y[indices], 'ro')
        self.ax.set_xlabel('X (metros)')
        self.ax.set_ylabel('Y (metros)')
        self.ax.set_title('Leituras do LIDAR')
        self.ax.axis('equal')
        self.ax.grid(True)
        plt.pause(0.1)
        plt.draw()
        self.get_logger().info('Mensagem Recebida: '+msg.header.frame_id + f' N_dados:{len(distances):.2f}')
        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    LidarTestSub = LidarTest()

    # faz o spin do nó para habilitar recebimento de callbacks
    rclpy.spin(LidarTestSub)

    # especificação de shutdown
    LidarTestSub.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()