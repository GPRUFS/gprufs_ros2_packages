import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from geometry_msgs.msg import Twist


class EncoderTest(Node):

    def __init__(self):
        super().__init__('encoder_test')
        self.subscription_ = self.create_subscription(Twist, '/robot/encoder', self.encoder_callback, 1)
        self.subscription_ # para previnir o "warning" de variável sem uso


    def encoder_callback(self, msg):
        self.get_logger().info(f'left speed: {msg.angular.x:.2f} | right speed: {msg.angular.y:.2f}')
        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    EncoderTestSub = EncoderTest()

    # faz o spin do nó para habilitar recebimento de callbacks
    rclpy.spin(EncoderTestSub)

    # especificação de shutdown
    EncoderTestSub.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()