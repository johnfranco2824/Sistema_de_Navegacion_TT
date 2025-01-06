#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

class StopPublisher(Node):
    def __init__(self):
        super().__init__('stop_publisher')
        self.publisher = self.create_publisher(Bool, '/stop', 10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.stop_signal = False  # Inicialmente no hay señal de parada

    def lidar_callback(self, msg):
        
        #Procesa los datos del LIDAR y publica una señal de parada si detecta un objeto a menos de 1 metro de distancia.
        range_min = msg.range_min
        range_max = msg.range_max

        # Recorremos las lecturas del LIDAR para detectar objetos cerca
        for distance in msg.ranges:
            if range_min <= distance <= range_max and distance < 1.0:
                self.stop_signal = True
                break
        else:
            self.stop_signal = False

        # Publicamos la señal de parada
        msg = Bool()
        msg.data = self.stop_signal
        self.publisher.publish(msg)

        if self.stop_signal:
            self.get_logger().info("Obstáculo detectado a menos de 1 metro. Señal de parada publicada.")
        else:
            self.get_logger().info("No se detectaron obstáculos cercanos.")

def main(args=None):
    rclpy.init(args=args)
    node = StopPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
