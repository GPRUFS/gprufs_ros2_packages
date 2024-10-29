import rclpy
import serial 
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from py_lidar.LidarX2 import LidarX2


class WalkPublisher(Node):

    def __init__(self):
        super().__init__('walk_publisher')
        self.publisher_lidar = self.create_publisher(LaserScan, '/robot/lidar', 1)
        #LidarX2
        self.lidarx2 = LidarX2("COM3")  # Name of the serial port, can be /dev/tty*, COM*, etc.
        self.lidar_msg = LaserScan()
        if not self.lidarx2.open():
            print("Cannot open lidarX2")
            exit(1)
        # -- Realiza as inscrições no tópico do 
        #Comunicação USB com o stm32
        self.stm32 = serial.Serial(port = 'COM9',baudrate=115200)
        self.vel = Twist()
        self.subscription = self.create_subscription(Twist, '/robot/cmd_vel', self.stm32_callback, 1)
        #Modelo cinemático do robô
        R = 5.3 #raio das rodas em cm
        d = 11.25/2 #distância entre as rodas e o centro do robô em cm
        self.modelo_cinematico = 0.5*np.array([[R,R],[R/d,- R/d]])
        self.modelo_cinematico_inverso = np.linalg.inv(self.modelo_cinematico)
        self.Linear_maximo = 5.3
        self.Angular_maximo = 0.9422
    
    def stm32_callback(self,msg):
        #self.sendMeasures()
        self.enviar_velocidade(msg)

    def enviar_velocidade(self, msg):

        V = np.array([[msg.linear.x,msg.angular.z]]).T #[v;W]
    
        if(np.abs(V[0]) > np.abs(self.Linear_maximo)):
            V[0] = self.Linear_maximo * V[0]/np.abs(V[0])

        if(np.abs(V[1]) > np.abs(self.Angular_maximo)):
            V[1] = self.Angular_maximo * V[1]/np.abs(V[1])

        FI = self.modelo_cinematico_inverso @ V #[fi_d;fi_e]

        msg2 = [255,0,0]
        if(FI[0] >= 0):  
            msg2[1] = round(100*np.abs(FI[0][0]))
        else:
            msg2[1] = -round(100*np.abs(FI[0][0])) + 127
        msg2[1] = int(msg2[1])
        if(FI[1] >= 0):
            msg2[2] = round(100*np.abs(FI[1][0]))
        else:
            msg2[2] = -round(100*np.abs(FI[1][0])) + 127
        msg2[2] = int(msg2[2])
        self.stm32.write(msg2)
        self.get_logger().info('Velocidades enviadas pela serial')

    def sendMeasures(self):
        measures = self.lidarx2.getMeasures()  
        if(len(measures) > 0): 
            self.lidar_msg.ranges,self.lidar_msg.angle_min,self.lidar_msg.angle_max = self.convert_to_scalar(measures)
        self.publisher_lidar.publish(self.lidar_msg)
        self.get_logger().info('Medidas Enviadas')

    def convert_to_scalar(self,measures):
        angles = []
        distances = []
        for measure in measures:
            angles.append(measure.angle*(np.pi/180))
            distances.append(measure.distance)

        return distances, np.min(angles), np.max(angles)
        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    walk_publisher = WalkPublisher()

    # Executando o a rotina de execução do nó. Esse código não possui 'callbacks'
    try:
        while(rclpy.ok()):
            walk_publisher.sendMeasures()
            # faz o spin do nó para habilitar recebimento de callbacks
            rclpy.spin_once(walk_publisher)

    except KeyboardInterrupt:
        walk_publisher.lidarx2.close()
        walk_publisher.destroy_node()
        rclpy.shutdown()
        print("Execução encerrada pelo usuário")
    


if __name__ == '__main__':
    main()