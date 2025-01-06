#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion
import math
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios


class DiffRobotController(Node):
    def __init__(self):
        super().__init__('diff_robot_controller')

        # Publicadores y suscriptores
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Pose2D, '/tracking_errors', 10)
        self.error_x_pub = self.create_publisher(Float64, '/error_x', 10)
        self.error_y_pub = self.create_publisher(Float64, '/error_y', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer para el bucle principal (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        # Inicialización de variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.V = 0.0
        self.W = 0.0
        self.ex = 0.0
        self.ey = 0.0
        self.t = 0.0
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.V_max = 2.0
        self.W_max = 5.0
        self.l = 0.1
        self.k = 0.15
        self.T = 100.0

        # Archivo de datos
        self.file_obj = open("data_py.txt", "w+")
        self.file_obj.write("Tiempo\tError_X\tError_Y\tPos_Xd\tPos_Yd\tXdp\tYdp\n")  # Encabezados

        self.counter = 0
        self.get_logger().info("Press 'q' or Ctrl+C to stop the robot.")

    def get_key(self):
        if os.name == 'nt':
            return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def velocity_controller(self):
        # Trayectoria deseada: Lemniscata
        a = 5
        b = 2.5
        X0 = 0
        Y0 = 0
        w = 2 * math.pi / self.T
        self.t = self.get_clock().now().nanoseconds / 1e9 - self.t0

        # Posición deseada
        Xd = X0 + a * math.sin(w * self.t)
        Yd = Y0 + b * math.sin(2 * w * self.t)

        # Derivadas temporales
        Xdp = a * w * math.cos(w * self.t)
        Ydp = 2 * b * w * math.cos(2 * w * self.t)

        # Coordenadas del punto externo
        p_x = self.x + self.l * math.cos(self.yaw)
        p_y = self.y + self.l * math.sin(self.yaw)

        # Errores de seguimiento
        self.ex = p_x - Xd
        self.ey = p_y - Yd

        # Control cinemático
        Ux = Xdp - self.k * self.ex
        Uy = Ydp - self.k * self.ey

        # Cálculo de velocidades
        self.V = Ux * math.cos(self.yaw) + Uy * math.sin(self.yaw)
        self.W = -Ux * math.sin(self.yaw) / self.l + Uy * math.cos(self.yaw) / self.l

        # Saturación de velocidades
        if abs(self.V) > self.V_max:
            self.V = self.V_max * abs(self.V) / self.V
            self.get_logger().warn("Linear velocity saturated")
        if abs(self.W) > self.W_max:
            self.W = self.W_max * abs(self.W) / self.W
            self.get_logger().warn("Angular velocity saturated")

        return Xd, Yd, Xdp, Ydp

    def control_loop(self):
        Xd, Yd, Xdp, Ydp = self.velocity_controller()

        # Publicar velocidades
        vel_msg = Twist()
        vel_msg.linear.x = self.V
        vel_msg.angular.z = self.W
        self.vel_pub.publish(vel_msg)

        # Publicar errores
        error_msg = Pose2D()
        error_msg.x = self.ex
        error_msg.y = self.ey
        self.error_pub.publish(error_msg)

        # Publicar errores individuales para rqt_plot
        error_x_msg = Float64()
        error_x_msg.data = self.ex
        self.error_x_pub.publish(error_x_msg)

        error_y_msg = Float64()
        error_y_msg.data = self.ey
        self.error_y_pub.publish(error_y_msg)

        # Guardar datos en archivo
        text_line = f"{self.t:.3f}\t{Xd:.3f}\t{Yd:.3f}\t{self.x:.3f}\t{self.y:.3f}\t{self.V:.3f}\t{self.W:.3f}\n"
        self.file_obj.write(text_line)

        # Mostrar datos en la consola cada 25 ciclos
        if self.counter == 25:
            self.get_logger().info(f"t: {self.t:.2f}, ex: {self.ex:.3f}, ey: {self.ey:.3f}, Xd: {Xd:.3f}, Yd: {Yd:.3f}")
            self.counter = 0
        else:
            self.counter += 1

        # Salir si se detecta la tecla 'q'
        key = self.get_key()
        if key == 'q' or key == '\x03':  # Ctrl+C
            self.stop_robot()
            self.file_obj.close()
            rclpy.shutdown()

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        self.get_logger().info("Robot stopped. Simulation continues.")


def main(args=None):
    if os.name != 'nt':
        global settings
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = DiffRobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.file_obj.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
