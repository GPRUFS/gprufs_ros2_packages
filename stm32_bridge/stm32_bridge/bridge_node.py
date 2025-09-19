#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import serial
import numpy as np

def bitset(valor, pos, val=1):
    if val:
        return valor | (1 << (pos - 1))
    else:
        return valor & ~(1 << (pos - 1))

def inverter_uL(x):
    return np.sign(x)*(np.abs(x) - (-0.414))/1.459

def inverter_uR(x):
    return np.sign(x)*(np.abs(x) - (-0.404))/1.423
def satura(x):
    if np.abs(x)>1:
        return np.sign(x)
    else:
        return x

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')
        self.get_logger().info('STM32 Bridge Node has been started.')

        #parâmetros de controle
        self.v_r = 0.0  # set point da velocidade da roda direita
        self.v_l = 0.0  # set point da velocidade da roda esquerda

        #parâmetros configuráveis
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheels_radius', 0.05)  # em metros
        self.declare_parameter('wheels_distance', 0.2)     # distância entre as rodas em metros

        #carregar parâmetros
        self.port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.L = self.get_parameter('wheels_distance').value
        self.R = self.get_parameter('wheels_radius').value

        self.get_logger().info(f'Parametros: port={self.port} baud={self.baud_rate}')

        self._open_serial()

        #Subscriber para receber comandos de velocidade
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            2 
            )
        
        #Publisher para enviar dados do STM32
        self.encoder_publisher = self.create_publisher(
            Float32MultiArray,
            '/encoder_data',
            2 
            )
        
        self.imu_pub = self.create_publisher(
        Imu,
        'imu',  
        2 
        )

        self.timer = self.create_timer(0.02, self.read_serial)  # 500 Hz (2 ms)

    def _open_serial(self): 
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Serial port {self.port} opened at {self.baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            self.serial = None

    def _close_serial(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.get_logger().info(f'Serial port {self.port} closed.')

    def cmd_vel_callback(self, msg):
        if self.serial and self.serial.is_open:
            v = msg.linear.x
            w = msg.angular.z
            self.v_r, self.v_l = self._inverse_kinematics(v, w) #set points de velocidade

    def read_serial(self):
        if self.serial and self.serial.is_open:
            msg = self._montar_mensagem(self.v_r, self.v_l)
            self.serial.write(msg)
            try:
                data = self.serial.read(20)  # Ler 20 bytes
                values = np.frombuffer(data, dtype=np.uint8)
                
                # publicando encoder
                encoder_msg = Float32MultiArray()
                encoder_msg.data = [values[2],values[3]] # encoder1 e encoder2
                self.encoder_publisher.publish(encoder_msg)

                # publicando IMU
                imu_msg = Imu()
                imu_msg.linear_acceleration.x = values[4]
                imu_msg.linear_acceleration.y = values[5]
                imu_msg.linear_acceleration.z = values[6]
                imu_msg.angular_velocity.x = values[7]
                imu_msg.angular_velocity.y = values[8]
                imu_msg.angular_velocity.z = values[9]
                self.imu_pub.publish(imu_msg)
            except Exception as e:
                self.get_logger().error(f'Error reading from serial: {e}')

    def _inverse_kinematics(self, v, w):
        v_r = (2 * v + w * self.L) / (2 * self.R)
        v_l = (2 * v - w * self.L) / (2 * self.R)
        return v_r, v_l
    
    def _montar_mensagem(self, v_r, v_l):
        Vmax = 35
        uL = v_l/Vmax #m/s ->  -1 a 1
        uR = v_r/Vmax #m/s ->  -1 a 1

        uL = satura(inverter_uL(uL))
        uR = satura(inverter_uR(uR))

        msg = [254, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        if uR >= 0:
            msg[8] = int(round(250 * abs(uR)))
            logica = bitset(logica, 2)
        else:
            msg[8] = int(round(250 * abs(uR)))
            logica = bitset(logica, 3)

        if uL >= 0:
            msg[9] = int(round(250 * abs(uL)))
            logica = bitset(logica, 4)
        else:
            msg[9] = int(round(250 * abs(uL)))
            logica = bitset(logica, 5)

        msg[7] = logica

        return bytes(msg)

def main(args=None):
        rclpy.init(args=args)
        node = STM32Bridge()
        try:
            rclpy.spin(node) # Mantém o nó ativo para callbacks
        except KeyboardInterrupt:
            pass
        finally: # o que fazer quando o nó for encerrado:
            node._close_serial()
            node.destroy_node()
            rclpy.shutdown()