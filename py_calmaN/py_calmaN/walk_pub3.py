import rclpy
import serial 
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import PyLidar3
from cv_bridge import CvBridge


class WalkPublisher(Node):

    def __init__(self):
        super().__init__('walk_publisher')
        self.publisher_lidar = self.create_publisher(LaserScan, '/robot/lidar', 1)
        #LidarX4
        self.lidarx4 = PyLidar3.YdLidarX4("/dev/ttyUSB0",2000)
        self.lidar_msg = LaserScan()
        if(not self.lidarx4.Connect()):
            print("Cannot open lidarX4")
            exit(1)
        print("Connected to lidarX4")
        self.lidarx4_read = self.lidarx4.StartScanning()
        # Configura tópico da câmera
        self.publisher_camera = self.create_publisher(Image, '/robot/camera', 1)
        # Configuração com a interface com a camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.camera.set(cv2.CAP_PROP_FPS,30)
        self.image = Image()
        self.bridge = CvBridge()
        # -- Realiza as inscrições no tópico do 
        #Comunicação USB com o stm32
        self.stm32 = serial.Serial(port = '/dev/ttyACM0',baudrate=115200)
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
        self.get_logger().info('Enviando ...')
        # Get latest lidar measures
        measures = next(self.lidarx4_read)
        if(len(measures) > 0): 
            self.lidar_msg.ranges,self.lidar_msg.angle_min,self.lidar_msg.angle_max = self.convert_to_scalar(measures)
        self.publisher_.publish(self.lidar_msg)
        self.get_logger().info('Medidas Enviadas')

    def convert_to_scalar(self,measures):
        angles = []
        distances = []
        for angle in measures:
            distance = float(measures[angle])
            angles.append(angle*(np.pi/180))
            distances.append(distance)
        return distances, np.min(angles), np.max(angles)
    
    def sendframe(self):
        ret, frame = self.camera.read()
        if(ret):
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            self.image = self.bridge.cv2_to_imgmsg(frame_gray,'mono8')
        self.publisher_camera.publish(self.image)
        self.get_logger().info('frame_enviado')        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    walk_publisher = WalkPublisher()

    # Executando o a rotina de execução do nó. Esse código não possui 'callbacks'
    try:
        while(rclpy.ok()):
            walk_publisher.sendMeasures()
            walk_publisher.sendframe()
            # faz o spin do nó para habilitar recebimento de callbacks
            rclpy.spin_once(walk_publisher)

    except KeyboardInterrupt:
        walk_publisher.lidarx2.close()
        walk_publisher.destroy_node()
        rclpy.shutdown()
        print("Execução encerrada pelo usuário")
    


if __name__ == '__main__':
    main()