import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math

class InclinationCalculator(Node):
    def __init__(self):
        super().__init__('inclination_calculator')
        
        # Assina o tópico /imu
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        
        # Cria um publisher para o tópico /inclination_angle
        self.publisher = self.create_publisher(
            Float32,
            '/inclination_angle',
            10)
        
        # Evita avisos de variável não utilizada
        self.subscription

    def imu_callback(self, msg):
        # Extrai os valores de aceleração linear
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Calcula a inclinação
        inclination = math.atan2(math.sqrt(ax**2 + ay**2), az)

        # Converte o ângulo para graus
        inclination_deg = math.degrees(inclination)

        # Cria uma mensagem Float32 para publicar o ângulo
        inclination_msg = Float32()
        inclination_msg.data = inclination_deg

        # Publica o ângulo no tópico /inclination_angle
        self.publisher.publish(inclination_msg)

        # Exibe o resultado no terminal (opcional)
        #self.get_logger().info(f'Inclination: {inclination_deg:.2f} graus')

def main(args=None):
    rclpy.init(args=args)
    inclination_calculator = InclinationCalculator()
    rclpy.spin(inclination_calculator)
    inclination_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
