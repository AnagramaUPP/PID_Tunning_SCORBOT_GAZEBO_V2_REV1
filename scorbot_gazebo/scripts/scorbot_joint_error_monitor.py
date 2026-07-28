#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray

import numpy as np


class ScorbotJointErrorMonitor(Node):

    def __init__(self):
        super().__init__('scorbot_joint_error_monitor')

        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5']

        self.q_real = [0.0] * 5
        self.q_desired = [0.0] * 5

        self.has_real = False
        self.has_desired = False

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.create_subscription(
            JointTrajectory,
            '/scorbot_arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.error_pub = self.create_publisher(
            Float64MultiArray,
            '/scorbot/joint_error',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_error)

    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q_real[i] = msg.position[idx]

        self.has_real = True

    def trajectory_callback(self, msg):
        if len(msg.points) == 0:
            return

        positions = msg.points[-1].positions

        if len(positions) != 5:
            return

        self.q_desired = list(positions)
        self.has_desired = True

    def publish_error(self):
        if not self.has_real or not self.has_desired:
            return

        error = [
            self.q_desired[i] - self.q_real[i]
            for i in range(5)
        ]

        error_norm = float(np.linalg.norm(error))

        msg = Float64MultiArray()
        msg.data = error + [error_norm]
        self.error_pub.publish(msg)

        self.get_logger().info(
            f'Error articular: '
            f'j1={error[0]:.4f}, '
            f'j2={error[1]:.4f}, '
            f'j3={error[2]:.4f}, '
            f'j4={error[3]:.4f}, '
            f'j5={error[4]:.4f}, '
            f'||e||={error_norm:.4f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScorbotJointErrorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()