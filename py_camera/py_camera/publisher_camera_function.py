import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/robot/camera', 1)
        # Configuração com a interface com a camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.camera.set(cv2.CAP_PROP_FPS,30)
        self.image = Image()
        self.bridge = CvBridge()

    def sendframe(self):
        ret, frame = self.camera.read()
        if(ret):
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            self.image = self.bridge.cv2_to_imgmsg(frame_gray,'mono8')
        self.publisher_.publish(self.image)
        self.get_logger().info('frame_enviado')
        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    camera_publisher = CameraPublisher()

    # Executando o a rotina de execução do nó. Esse código não possui 'callbacks'
    try:
        while(rclpy.ok()):
            camera_publisher.sendframe()

    except KeyboardInterrupt:
        camera_publisher.destroy_node()
        rclpy.shutdown()
        print("Execução encerrada pelo usuário")
    


if __name__ == '__main__':
    main()