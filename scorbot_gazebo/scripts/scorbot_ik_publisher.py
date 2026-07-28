#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np


def CI(Pefx, Pefy, Pefz, delta):
    L1 = 0.3520
    L2 = -0.02437
    L3 = 0.220
    L4 = 0.220
    L5 = 0.1479

    Phiy = np.pi / 2
    Phiz = np.pi
    gamma = np.pi

    Theta1 = np.arctan2(Pefy, Pefx)

    Ry = np.array([
        [np.cos(Phiy), 0, np.sin(Phiy)],
        [0, 1, 0],
        [-np.sin(Phiy), 0, np.cos(Phiy)]
    ])

    Rz = np.array([
        [np.cos(Phiz), -np.sin(Phiz), 0],
        [np.sin(Phiz),  np.cos(Phiz), 0],
        [0, 0, 1]
    ])

    RT = np.array([
        [1, 0, 0],
        [0, np.cos(Theta1), -np.sin(Theta1)],
        [0, np.sin(Theta1),  np.cos(Theta1)]
    ])

    RDelta = np.array([
        [np.cos(delta), 0, np.sin(delta)],
        [0, 1, 0],
        [-np.sin(delta), 0, np.cos(delta)]
    ])

    Rgamma = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma),  np.cos(gamma), 0],
        [0, 0, 1]
    ])

    R6_0 = Ry @ Rz @ RT @ RDelta @ Rgamma
    R6_5 = np.eye(3)
    R5_0 = R6_0 @ R6_5.T

    P6_0 = np.array([[Pefx], [Pefy], [Pefz]])
    P6_5 = np.array([[0], [0], [L5]])

    PMu_0 = P6_0 - R5_0 @ P6_5

    Xc = PMu_0[0]
    Yc = PMu_0[1]
    Zc = PMu_0[2]

    R = np.sqrt(Xc**2 + Yc**2)
    h = Zc - L1
    m = np.sqrt(h**2 + (R - L2)**2)

    alpha = np.arctan2(h, (R - L2))

    beta_cos = (m**2 + L3**2 - L4**2) / (2 * m * L3)
    beta_cos = np.clip(beta_cos, -1.0, 1.0)

    beta = np.arctan2(np.sqrt(1 - beta_cos**2), beta_cos)

    Theta2 = alpha + beta
    Theta2 = float(Theta2[0])

    phi_cos = (L3**2 + L4**2 - m**2) / (2 * L3 * L4)
    phi_cos = np.clip(phi_cos, -1.0, 1.0)

    phi = np.arctan2(np.sqrt(1 - phi_cos**2), phi_cos)

    Theta3 = -np.pi + phi
    Theta3 = float(Theta3[0])
    
    R1_0 = np.array([
        [np.cos(Theta1), -np.sin(Theta1), 0],
        [np.sin(Theta1),  np.cos(Theta1), 0],
        [0, 0, 1]
    ])

    R2_1 = np.array([
        [np.cos(Theta2), -np.sin(Theta2), 0],
        [0, 0, -1],
        [np.sin(Theta2),  np.cos(Theta2), 0]
    ])

    R3_2 = np.array([
        [np.cos(Theta3), -np.sin(Theta3), 0],
        [np.sin(Theta3),  np.cos(Theta3), 0],
        [0, 0, 1]
    ])

    R3_0 = R1_0 @ R2_1 @ R3_2
    R6_3 = R3_0.T @ R6_0 @ R6_5.T

    Theta5 = np.arctan2(
        np.sqrt(R6_3[2, 0]**2 + R6_3[2, 1]**2),
        R6_3[2, 2]
    )

    Theta4 = np.arctan2(R6_3[1, 2], R6_3[0, 2])

    return float(Theta1), float(Theta2), float(Theta3), float(Theta4), float(Theta5)


class ScorbotIKPublisher(Node):

    def __init__(self):
        super().__init__('scorbot_ik_publisher')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/scorbot_arm_controller/joint_trajectory',
            10
        )

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/scorbot/desired_pose',
            self.desired_pose_callback,
            10
        )

        self.Pefx = 0.51579628
        self.Pefy = 0.0
        self.Pefz = 0.2718165
        self.delta = 0.0

        self.timer = self.create_timer(
            0.1,
            self.publish_joint_trajectory
        )

    def desired_pose_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(
                'El mensaje debe tener 4 valores: [x, y, z, delta]'
            )
            return

        self.Pefx = msg.data[0]
        self.Pefy = msg.data[1]
        self.Pefz = msg.data[2]
        self.delta = msg.data[3]

    def publish_joint_trajectory(self):
        theta1, theta2, theta3, theta4, theta5 = CI(
            self.Pefx,
            self.Pefy,
            self.Pefz,
            self.delta
        )

        msg = JointTrajectory()
        msg.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5']

        point = JointTrajectoryPoint()
        point.positions = [
            theta1,
            theta2,
            theta3,
            theta4,
            theta5
        ]

        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(
            f'JointTrajectory enviado: '
            f'[{theta1:.3f}, {theta2:.3f}, {theta3:.3f}, {theta4:.3f}, {theta5:.3f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScorbotIKPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()