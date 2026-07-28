#!/usr/bin/env python3

import csv
import math
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener


class CartesianTrackingLogger(Node):

    def __init__(self):
        super().__init__('cartesian_tracking_logger')

        self.xd = None
        self.yd = None
        self.zd = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.csv_path = os.path.expanduser(
            '~/ros2_ws/scorbot_tracking.csv'
        )

        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time',
            'xd', 'yd', 'zd',
            'xr', 'yr', 'zr',
            'ex', 'ey', 'ez',
            'error_norm'
        ])

        self.error_pub = self.create_publisher(
            Float64MultiArray,
            '/scorbot/cartesian_error',
            10
        )

        self.tracking_pub = self.create_publisher(
            Float64MultiArray,
            '/scorbot/cartesian_tracking',
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/scorbot/desired_pose',
            self.desired_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.log_tracking
        )

        self.get_logger().info(
            f'Guardando datos en: {self.csv_path}'
        )

    def desired_callback(self, msg):
        if len(msg.data) < 3:
            return

        self.xd = msg.data[0]
        self.yd = msg.data[1]
        self.zd = msg.data[2]

    def log_tracking(self):
        if self.xd is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool_link',
                rclpy.time.Time()
            )

            xr = transform.transform.translation.x
            yr = transform.transform.translation.y
            zr = transform.transform.translation.z

            ex = self.xd - xr
            ey = self.yd - yr
            ez = self.zd - zr

            error_norm = math.sqrt(
                ex**2 + ey**2 + ez**2
            )

            t = self.get_clock().now().nanoseconds / 1e9

            self.writer.writerow([
                t,
                self.xd, self.yd, self.zd,
                xr, yr, zr,
                ex, ey, ez,
                error_norm
            ])

            self.file.flush()

            error_msg = Float64MultiArray()
            error_msg.data = [
                ex,
                ey,
                ez,
                error_norm
            ]
            self.error_pub.publish(error_msg)

            tracking_msg = Float64MultiArray()
            tracking_msg.data = [
                self.xd, self.yd, self.zd,
                xr, yr, zr,
                ex, ey, ez,
                error_norm
            ]
            self.tracking_pub.publish(tracking_msg)

            self.get_logger().info(
                f'e_cart={error_norm:.4f} m | '
                f'xd=({self.xd:.3f},{self.yd:.3f},{self.zd:.3f}) '
                f'xr=({xr:.3f},{yr:.3f},{zr:.3f})'
            )

        except Exception as e:
            self.get_logger().warn(
                f'No se pudo obtener TF base_link -> link_5: {e}'
            )

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CartesianTrackingLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()