import rclpy
import cv2
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription_ = self.create_subscription(Image, '/robot/camera', self.camera_callback, 1)
        self.subscription_ # para previnir o "warning" de variável sem uso
        self.bridge = CvBridge()
        self.last_time = time.time()
        self.fps = 0.0
        self.fps_file = open("D:/fps_data.txt", "w")  # Abre o arquivo para escrita

    def camera_callback(self, msg):
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        self.last_time = current_time

        if elapsed_time > 0:
            self.fps = 1.0 / elapsed_time

        # Escreve o valor de FPS no arquivo
        self.fps_file.write(f"{self.fps:.2f}\n")
        
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.putText(frame, f'FPS: {self.fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)
        self.get_logger().info(f'Frame received - FPS: {self.fps:.2f}')
        
    def destroy(self):
        self.fps_file.close()  # Fecha o arquivo ao destruir o nó

def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    camera_subscriber = CameraSubscriber()

    # faz o spin do nó para habilitar recebimento de callbacks
    rclpy.spin(camera_subscriber)

    # especificação de shutdown
    camera_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    


if __name__ == '__main__':
    main()