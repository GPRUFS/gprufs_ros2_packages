import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from py_lidar.LidarX2 import LidarX2


class LidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/robot/lidar', 1)
        #LidarX2
        self.lidarx2 = LidarX2("COM3")  # Name of the serial port, can be /dev/tty*, COM*, etc.
        self.lidar_msg = LaserScan()
        if not self.lidarx2.open():
            print("Cannot open lidarX2")
            exit(1)

    def sendMeasures(self):
        measures = self.lidarx2.getMeasures()  
        if(len(measures) > 0): 
            self.lidar_msg.ranges,self.lidar_msg.angle_min,self.lidar_msg.angle_max = self.convert_to_scalar(measures)
        self.publisher_.publish(self.lidar_msg)
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
    lidar_publisher = LidarPublisher()

    # Executando o a rotina de execução do nó. Esse código não possui 'callbacks'
    try:
        while(rclpy.ok()):
            lidar_publisher.sendMeasures()

    except KeyboardInterrupt:
        lidar_publisher.lidarx2.close()
        lidar_publisher.destroy_node()
        rclpy.shutdown()
        print("Execução encerrada pelo usuário")
    


if __name__ == '__main__':
    main()