import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import PyLidar3


class LidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/robot/lidar', 1)
        #LidarX4
        self.lidarx4 = PyLidar3.YdLidarX4("/dev/ttyUSB0",2000)
        self.lidar_msg = LaserScan()
        if(not self.lidarx4.Connect()):
            print("Cannot open lidarX4")
            exit(1)
        print("Connected to lidarX4")
        self.lidarx4_read = self.lidarx4.StartScanning()

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


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    lidar_publisher = LidarPublisher()

    # Executando o a rotina de execução do nó. Esse código não possui 'callbacks'
    try:
        while(rclpy.ok()):
            lidar_publisher.sendMeasures()

    except KeyboardInterrupt:
        lidar_publisher.lidarx4.StopScanning()
        lidar_publisher.lidarx4.Disconnect()
        lidar_publisher.destroy_node()
        rclpy.shutdown()
        print("Execução encerrada pelo usuário")
    


if __name__ == '__main__':
    main()