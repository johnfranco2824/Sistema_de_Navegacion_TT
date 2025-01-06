#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time


class LidarScanListener(Node):
    def __init__(self):
        super().__init__('lidar_sensor')  # Nombre del nodo

        # Suscripción al tópico /scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Inicialización del tiempo
        self.t0 = time.time()
        self.get_logger().info("Nodo iniciado. Presiona 'Ctrl+C' para detenerlo.")

    def scan_callback(self, msg):
        #Callback para procesar las mediciones del LIDAR.
        t = time.time() - self.t0  # Tiempo transcurrido
        self.get_logger().info(f"Tiempo: {t:.2f}s")

        range_size = len(msg.ranges)
        range_min = msg.range_min
        range_max = msg.range_max

        self.get_logger().info(f"Len(ranges): {range_size}")
        self.get_logger().info(f"Range Min: {range_min}, Range Max: {range_max}")

        # Procesar las lecturas del LIDAR
        for i, distance in enumerate(msg.ranges):
            if range_min <= distance <= range_max:
                self.get_logger().info(f"i: {i}, Distancia: {distance:.2f}")


def main(args=None):
    rclpy.init(args=args)  # Inicializar ROS 2
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
