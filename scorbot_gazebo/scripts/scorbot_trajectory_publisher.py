#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
import numpy as np


class ScorbotTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('scorbot_trajectory_publisher')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/scorbot/desired_pose',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/scorbot/trajectory_marker',
            10
        )

        self.real_marker_pub = self.create_publisher(
            Marker,
            '/scorbot/real_trajectory_marker',
            10
        )

        # Posición inicial del efector final
        self.x0 = 0.51579628
        self.y0 = 0.0
        self.z0 = 0.2718165
        self.delta = 0

        # Tamaño de la lemniscata
        self.A = 0.16
        self.B = 0.16

        # Variable de tiempo paramétrico
        self.t = 0.0

        # Incremento de avance
        self.dt = 0.002

        # =====================================================
        # Marker rojo: trayectoria deseada
        # =====================================================

        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.ns = "trajectory"
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.005

        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        # =====================================================
        # Marker azul: trayectoria real del efector final link_5
        # =====================================================

        self.real_marker = Marker()
        self.real_marker.header.frame_id = "base_link"
        self.real_marker.ns = "real_trajectory"
        self.real_marker.id = 1
        self.real_marker.type = Marker.LINE_STRIP
        self.real_marker.action = Marker.ADD
        self.real_marker.scale.x = 0.005

        self.real_marker.color.r = 0.0
        self.real_marker.color.g = 0.0
        self.real_marker.color.b = 1.0
        self.real_marker.color.a = 1.0

        # Buffer y listener para leer TF base_link -> link_5
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publicación a 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_trajectory)

    def publish_trajectory(self):

        # =====================================================
        # Lemniscata en el plano YZ
        # =====================================================

        x = self.x0
        y = self.y0 + self.A * np.sin(self.t)
        z = self.z0 + self.B * np.sin(self.t) * np.cos(self.t)

        # =====================================================
        # Publicar la posición deseada en /scorbot/desired_pose
        # =====================================================

        msg = Float64MultiArray()
        msg.data = [
            float(x),
            float(y),
            float(z),
            float(self.delta)
        ]

        self.publisher.publish(msg)

        # =====================================================
        # Marker rojo: trayectoria deseada
        # =====================================================

        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)

        self.marker.points.append(p)
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)

        # =====================================================
        # Marker azul: trayectoria real desde TF base_link -> link_5
        # =====================================================

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'link_5',
                rclpy.time.Time()
            )

            p_real = Point()
            p_real.x = transform.transform.translation.x
            p_real.y = transform.transform.translation.y
            p_real.z = transform.transform.translation.z

            self.real_marker.points.append(p_real)
            self.real_marker.header.stamp = self.get_clock().now().to_msg()
            self.real_marker_pub.publish(self.real_marker)

        except Exception as e:
            self.get_logger().warn(
                f'No se pudo obtener TF base_link -> link_5: {e}'
            )

        self.get_logger().info(
            f'Trayectoria YZ: '
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'z={z:.3f}, '
            f'delta={self.delta:.3f}'
        )

        self.t += self.dt

        if self.t >= 2 * np.pi:
            self.t = 0.0


def main(args=None):
    rclpy.init(args=args)

    node = ScorbotTrajectoryPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()