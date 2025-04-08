import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist


class Modelagem(Node):

    def __init__(self):
        super().__init__('ModelExp')
        # -- Realiza as inscrições nos tópicos dos sensores
        self.subscription = self.create_subscription(Int32MultiArray, '/robot/encoder', self.encoder_callback, 1)
        self.subscription # para previnir o "warning" de variável sem uso
        # -- Definição do Publisher das velocidades
        self.velPub = self.create_publisher(Twist, '/robot/cmd_vel', 1)
        timer_period = 3.333*1e-3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # -- Inicialização das variáveis
        self.encoder_data = Int32MultiArray()
        self.start_flag = False
        self.cmdVel = np.concatenate((np.linspace(-1,1,21),np.zeros(21)))
        self.idc = 0
        self.contador = 0
        self.vL = []
        self.vR = []
        self.uL = []
        self.uR = []
        self.d = Twist()

    def timer_callback(self):

        if(self.start_flag): 
            
            # Publica as velocidades
            self.d.angular.x = self.cmdVel[self.idc]
            self.d.angular.y = self.cmdVel[41-self.idc]
            self.velPub.publish(self.d)

            if self.contador>=600:
                self.idc+=1
                self.contador=0
                self.start_flag=False

        else:
            self.d.angular.x = 0.0
            self.d.angular.y = 0.0
            self.velPub.publish(self.d)

            if self.contador>=600:
                self.contador=0
                self.start_flag=True
            # Log

        if self.idc==42:

            
            self.d.angular.x = 0.0
            self.d.angular.y = 0.0
            self.velPub.publish(self.d)

            plt.figure()
            plt.subplot(2,1,1)
            plt.plot(self.vL)
            plt.title("Roda Esquerda")
            plt.subplot(2,1,2)
            plt.plot(self.vR)
            plt.title("Roda Direita")
            plt.show()

            self.get_logger().info('Salvando os dados')
            with open("dados.txt","w") as f:
                f.write(",".join((map(str,self.uL)))+"\n")
                f.write(",".join((map(str,self.vL)))+"\n")
                f.write(",".join((map(str,self.uR)))+"\n")
                f.write(",".join((map(str,self.vR)))+"\n")
            self.get_logger().info('Dados salvos')
            exit(1)



        self.contador = self.contador + 1
        self.get_logger().info(f'VelLeft: {self.d.angular.x:.2f} | VelRight: {self.d.angular.y:.2f}')


    def encoder_callback(self, msg):
        self.vL.append(msg.data[2])
        self.vR.append(msg.data[3])
        self.uL.append(self.d.angular.x)
        self.uR.append(self.d.angular.y)
        self.get_logger().info('Atualizando dados')
        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    ModelSub = Modelagem()

    # faz o spin do nó para habilitar recebimento de callbacks
    rclpy.spin(ModelSub)

    # especificação de shutdown
    ModelSub.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
