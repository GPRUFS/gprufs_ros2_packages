import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


class Walk(Node):

    def __init__(self):
        super().__init__('Walker')
        self.subscription_ = self.create_subscription(Bool, '/robot/sensorTrigger', self.Controller_callback, 1)
        self.subscription_ # para previnir o "warning" de variável sem uso
        self.leftMotorPub = self.create_publisher(Float32, '/robot/leftMotorSpeed', 1)
        self.rightMotorPub = self.create_publisher(Float32, '/robot/rightMotorSpeed', 1)


    def Controller_callback(self, msg):

        if(msg.data==True):
            desiredLeftSpeed = -7.0*0.25
            desiredRightSpeed = 7.0*0.25
        else:
            desiredLeftSpeed = 7.0
            desiredRightSpeed = 7.0

        d = Float32()
        d.data = desiredLeftSpeed
        self.leftMotorPub.publish(d)
        d.data = desiredRightSpeed
        self.rightMotorPub.publish(d)
        self.get_logger().info(f'Left: {desiredLeftSpeed:.2f} | Right: {desiredRightSpeed:.2f}')
        


def main(args=None):


    # Inicilaização
    rclpy.init(args=args)

    # Criação de um nó instanciando uma classe do tipo "Node", por meio de "hierarquia"
    WalkSub = Walk()

    # faz o spin do nó para habilitar recebimento de callbacks
    rclpy.spin(WalkSub)

    # especificação de shutdown
    WalkSub.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()