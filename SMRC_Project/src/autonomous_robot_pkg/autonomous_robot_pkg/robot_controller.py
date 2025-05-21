import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()

    node = rclpy.create_node('robot_controller')

    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    msg = Twist()
    vx = 0.0
    vy = 0.0
    vz = 0.0

    wx = 0.0
    wy = 0.0
    wz = -0.5
    msg.linear.x = vx  # Velocidade linear em m/s
    msg.angular.z = wz  # Velocidade angular em rad/s

    tf = 50  # Tempo total de movimento em segundos
    start_time = time.time()

    while time.time() - start_time < tf:
        t = time.time() - start_time
        publisher.publish(msg)
        #node.get_logger().info('Velocidade linear x = %f', vx)
        print('Tempo =', t)
        print('vx =', vx)
        print('vy =', vy)
        print('vz =', vz)
        print('wx =', wx)
        print('wy =', wy)
        print('wz =', wz)

        time.sleep(0.1)  # Frequência de publicação

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

