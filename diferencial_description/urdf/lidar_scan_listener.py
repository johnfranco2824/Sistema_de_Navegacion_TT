#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class LidarScanListener(Node):
    def __init__(self):
        super().__init__('lidar_scan_listening')  # Nombre del nodo

        # Suscripción al tópico /scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Publicador para detener el robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_pub = self.create_publisher(Bool, '/stop', 10)

        # Inicialización del tiempo
        self.t0 = time.time()
        self.get_logger().info("Nodo iniciado. Presiona 'Ctrl+C' para detenerlo.")

    def scan_callback(self, msg):
        # Variables para rango mínimo y máximo del LIDAR
        range_min = msg.range_min
        range_max = msg.range_max

        # Procesar las lecturas del LIDAR
        obstacle_detected = False
        for i, distance in enumerate(msg.ranges):
            if range_min <= distance <= range_max and distance < 1.0:  # Obstáculo a menos de 1 metro
                obstacle_detected = True
                self.get_logger().warn(f"Obstáculo detectado en i: {i}, Distancia: {distance:.2f}m")
                break

        # Publicar señal de parada
        stop_msg = Bool()
        stop_msg.data = obstacle_detected
        self.stop_pub.publish(stop_msg)

        # Detener el robot si hay un obstáculo
        if obstacle_detected:
            self.stop_robot()

        # Mostrar el tiempo transcurrido
        t = time.time() - self.t0
        self.get_logger().info(f"Tiempo: {t:.2f}s")

    def stop_robot(self):
        # Publica velocidades cero para detener el robot.
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("Robot detenido debido a un obstáculo.")


def main(args=None):
    rclpy.init(args=args)  # Inicializar
    node = LidarScanListener()  # Crear el nodo

    try:
        rclpy.spin(node)  # Mantener el nodo corriendo
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\nNodo finalizado.")


if __name__ == '__main__':
    main()

