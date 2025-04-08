import rclpy
import numpy as np
import pygame
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Joy(Node):

    def __init__(self):
        super().__init__('Joystick')
        # -- Realiza as inscrições nos tópicos dos sensores
        self.subscription = self.create_subscription(LaserScan, '/robot/lidar', self.lidar_callback, 1)
        self.subscription # para previnir o "warning" de variável sem uso
        # -- Definição do Publisher das velocidades
        self.velPub = self.create_publisher(Twist, '/robot/cmd_vel', 1)
        timer_period = 3.333*1e-3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # -- Inicialização das variáveis
        self.laser_scan_data = LaserScan()
        self.encoder_data = Twist()
        self.start_flag = False
        # Inicializa o pygame
        pygame.init()
        # Inicializa o joystick
        pygame.joystick.init()
        # Pega o primeiro joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        # Ativar modo interativo
        plt.ion()  
        self.fig, self.ax = plt.subplots()

    def timer_callback(self):

        if(self.start_flag): # Controlador

            pygame.event.pump()  # Atualiza eventos do pygame

            eixo_x = self.joystick.get_axis(0)  # Eixo X do analógico esquerdo
            eixo_y = self.joystick.get_axis(1)  # Eixo Y do analógico esquerdo

            VL = 0.5*(-eixo_y + eixo_x)
            VR = 0.5*(-eixo_y - eixo_x)
            self.get_logger().info(f'VelLeft: {VL:.2f} | VelRight: {VR:.2f}')

            #VL = 0.0
            #VR = 0.0
            # Publica as velocidades
            d = Twist()
            d.angular.x = VL
            d.angular.y = VR
            self.velPub.publish(d)


        else:
            self.get_logger().info('Esperando receber a primeira mensagem')
            d = Twist()
            d.angular.x = 0.0
            d.angular.y = 0.0
            self.velPub.publish(d)
            # Log
            self.get_logger().info(f'VelLeft: {d.angular.x:.2f} | VelRight: {d.angular.y:.2f}')


    def lidar_callback(self, msg):
        self.start_flag = True
        self.laser_scan_data = msg
        self.get_logger().info('Atualizando dados')
        distances = np.array(msg.ranges)
        idc = (np.where(distances > 0))
        angles = np.linspace(msg.angle_min, msg.angle_max, len(distances))
        angles = angles[idc]
        distances = distances[idc]

        x = distances * np.cos(angles-(np.pi/2.0))
        y = distances * np.sin(angles-(np.pi/2.0))
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
        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    JoySub = Joy()

    # faz o spin do nó para habilitar recebimento de callbacks
    rclpy.spin(JoySub)

    # especificação de shutdown
    JoySub.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
