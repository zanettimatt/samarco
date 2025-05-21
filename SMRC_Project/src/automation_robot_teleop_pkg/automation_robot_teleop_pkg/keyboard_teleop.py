#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import tty
import sys
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Configurações de velocidade
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.current_key = None  # Tecla atualmente pressionada
        
        self.get_logger().info("""
Controle o robô pelo teclado:
---------------------------
Teclas:
    ↑
←   ↓   →   (setas direcionais)

q/z : aumenta/diminui velocidade linear em 10%
w/x : aumenta/diminui velocidade angular em 10%

ESPAÇO: para o robô
CTRL+C: sair
""")

    def get_key(self):
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                if key == '\x1b':  # Tecla de escape
                    key += sys.stdin.read(2)  # Lê os próximos 2 caracteres para setas
                return key
            return None  # Retorna None quando nenhuma tecla está pressionada
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Atualiza a tecla atual apenas se uma nova tecla foi pressionada
                if key is not None:
                    self.current_key = key
                
                # Se nenhuma tecla está sendo pressionada, para o robô
                if key is None and self.current_key is not None:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.current_key = None
                elif self.current_key == '\x03':  # CTRL+C
                    break
                elif self.current_key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif self.current_key == '\x1b[A':  # Seta para cima
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                elif self.current_key == '\x1b[B':  # Seta para baixo
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = 0.0
                elif self.current_key == '\x1b[C':  # Seta para direita
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed
                elif self.current_key == '\x1b[D':  # Seta para esquerda
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed
                elif self.current_key == 'q':
                    self.linear_speed = min(self.linear_speed + 0.1, 2.0)
                    self.get_logger().info(f"Velocidade linear: {self.linear_speed} m/s")
                    continue  # Não publica movimento, apenas ajusta velocidade
                elif self.current_key == 'z':
                    self.linear_speed = max(self.linear_speed - 0.1, 0.1)
                    self.get_logger().info(f"Velocidade linear: {self.linear_speed} m/s")
                    continue  # Não publica movimento, apenas ajusta velocidade
                elif self.current_key == 'w':
                    self.angular_speed = min(self.angular_speed + 0.1, 2.0)
                    self.get_logger().info(f"Velocidade angular: {self.angular_speed} rad/s")
                    continue  # Não publica movimento, apenas ajusta velocidade
                elif self.current_key == 'x':
                    self.angular_speed = max(self.angular_speed - 0.1, 0.1)
                    self.get_logger().info(f"Velocidade angular: {self.angular_speed} rad/s")
                    continue  # Não publica movimento, apenas ajusta velocidade
                
                self.publisher_.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f"Erro: {e}")
        finally:
            # Para o robô antes de sair
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
