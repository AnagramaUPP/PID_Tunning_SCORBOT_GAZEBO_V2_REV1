#!/usr/bin/env python3

"""
Genera una vuelta completa de la lemniscata del Scorbot.

La trayectoria cartesiana se obtiene del paquete scorbot_gazebo
y la trayectoria articular se calcula mediante su función CI().
"""

from pathlib import Path

import numpy as np


# ============================================================
# Parámetros originales de la lemniscata
# ============================================================

X0 = 0.51579628
Y0 = 0.0
Z0 = 0.2718165
DELTA = 0.0

A = 0.16
B = 0.16

NUMBER_OF_POINTS = 200


def CI(Pefx, Pefy, Pefz, delta):
    """
    Cinemática inversa tomada de scorbot_ik_publisher.py.
    """
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

    P6_0 = np.array([
        [Pefx],
        [Pefy],
        [Pefz]
    ])

    P6_5 = np.array([
        [0],
        [0],
        [L5]
    ])

    PMu_0 = P6_0 - R5_0 @ P6_5

    Xc = PMu_0[0]
    Yc = PMu_0[1]
    Zc = PMu_0[2]

    R = np.sqrt(Xc**2 + Yc**2)
    h = Zc - L1
    m = np.sqrt(h**2 + (R - L2)**2)

    m_value = float(m[0])

    minimum_reach = abs(L3 - L4)
    maximum_reach = L3 + L4

    if not (
        minimum_reach - 1e-9
        <= m_value
        <= maximum_reach + 1e-9
    ):
        raise ValueError(
            'Punto fuera del espacio alcanzable: '
            f'P=({Pefx}, {Pefy}, {Pefz}), '
            f'm={m_value}'
        )

    alpha = np.arctan2(
        h,
        R - L2
    )

    beta_cos = (
        m**2 + L3**2 - L4**2
    ) / (
        2 * m * L3
    )

    beta_cos = np.clip(
        beta_cos,
        -1.0,
        1.0
    )

    beta = np.arctan2(
        np.sqrt(1 - beta_cos**2),
        beta_cos
    )

    Theta2 = alpha + beta
    Theta2 = float(Theta2[0])

    phi_cos = (
        L3**2 + L4**2 - m**2
    ) / (
        2 * L3 * L4
    )

    phi_cos = np.clip(
        phi_cos,
        -1.0,
        1.0
    )

    phi = np.arctan2(
        np.sqrt(1 - phi_cos**2),
        phi_cos
    )

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
        np.sqrt(
            R6_3[2, 0]**2
            + R6_3[2, 1]**2
        ),
        R6_3[2, 2]
    )

    Theta4 = np.arctan2(
        R6_3[1, 2],
        R6_3[0, 2]
    )

    return (
        float(Theta1),
        float(Theta2),
        float(Theta3),
        float(Theta4),
        float(Theta5),
        m_value,
    )


def main():
    package_root = (
        Path.home()
        / 'ros2_ws/src/scorbot_gazebo'
    )

    trajectories_directory = (
        package_root
        / 'trajectories'
    )

    trajectories_directory.mkdir(
        parents=True,
        exist_ok=True
    )

    cartesian_path = (
        trajectories_directory
        / 'cartesian_trajectory.csv'
    )

    joints_path = (
        trajectories_directory
        / 'joints_trajectory.csv'
    )

    # 200 muestras para una vuelta completa.
    t_values = np.linspace(
        0.0,
        2.0 * np.pi,
        NUMBER_OF_POINTS,
        endpoint=False,
        dtype=np.float64,
    )

    x_values = np.full(
        NUMBER_OF_POINTS,
        X0,
        dtype=np.float64,
    )

    y_values = (
        Y0
        + A * np.sin(t_values)
    )

    z_values = (
        Z0
        + B
        * np.sin(t_values)
        * np.cos(t_values)
    )

    cartesian_trajectory = np.column_stack([
        x_values,
        y_values,
        z_values,
    ])

    joint_trajectory = np.zeros(
        (NUMBER_OF_POINTS, 5),
        dtype=np.float64,
    )

    wrist_distances = np.zeros(
        NUMBER_OF_POINTS,
        dtype=np.float64,
    )

    for index, point in enumerate(cartesian_trajectory):
        result = CI(
            Pefx=point[0],
            Pefy=point[1],
            Pefz=point[2],
            delta=DELTA,
        )

        joint_trajectory[index, :] = result[:5]
        wrist_distances[index] = result[5]

    # ========================================================
    # Validaciones
    # ========================================================

    if cartesian_trajectory.shape != (200, 3):
        raise RuntimeError(
            f'Forma cartesiana incorrecta: '
            f'{cartesian_trajectory.shape}'
        )

    if joint_trajectory.shape != (200, 5):
        raise RuntimeError(
            f'Forma articular incorrecta: '
            f'{joint_trajectory.shape}'
        )

    if not np.all(
        np.isfinite(cartesian_trajectory)
    ):
        raise RuntimeError(
            'La trayectoria cartesiana contiene NaN o Inf.'
        )

    if not np.all(
        np.isfinite(joint_trajectory)
    ):
        raise RuntimeError(
            'La trayectoria articular contiene NaN o Inf.'
        )

    expected_first_point = np.array([
        X0,
        Y0,
        Z0,
    ])

    if not np.allclose(
        cartesian_trajectory[0],
        expected_first_point,
        atol=1e-12,
    ):
        raise RuntimeError(
            'El primer punto cartesiano no coincide '
            'con la pose inicial.'
        )

    # Saltos entre puntos consecutivos, incluyendo el cierre
    # desde el último punto hacia el primero.
    closed_joint_trajectory = np.vstack([
        joint_trajectory,
        joint_trajectory[0],
    ])

    maximum_joint_steps = np.max(
        np.abs(
            np.diff(
                closed_joint_trajectory,
                axis=0,
            )
        ),
        axis=0,
    )

    if np.any(maximum_joint_steps > 0.25):
        raise RuntimeError(
            'Se detectó un salto articular superior '
            f'a 0.25 rad: {maximum_joint_steps}'
        )

    # ========================================================
    # Guardar archivos
    # ========================================================

    np.savetxt(
        cartesian_path,
        cartesian_trajectory,
        delimiter=',',
        header='X,Y,Z',
        comments='',
        fmt='%.9f',
    )

    np.savetxt(
        joints_path,
        joint_trajectory,
        delimiter=',',
        header='q1,q2,q3,q4,q5',
        comments='',
        fmt='%.9f',
    )

    print()
    print('========================================')
    print('TRAYECTORIA GENERADA CORRECTAMENTE')
    print('========================================')

    print()
    print('Archivo cartesiano:')
    print(cartesian_path)

    print()
    print('Archivo articular:')
    print(joints_path)

    print()
    print('Número de puntos:')
    print(NUMBER_OF_POINTS)

    print()
    print('Primer punto cartesiano:')
    print(cartesian_trajectory[0])

    print()
    print('Primera pose articular calculada por CI:')
    print(joint_trajectory[0])

    print()
    print('Pose de aparición del URDF:')
    print(
        np.array([
            0.0,
            0.349,
            -0.75,
            0.0,
            1.57,
        ])
    )

    print()
    print('Rango articular de la trayectoria:')

    for joint_index in range(5):
        print(
            f'j{joint_index + 1}: '
            f'mín={joint_trajectory[:, joint_index].min():.9f}, '
            f'máx={joint_trajectory[:, joint_index].max():.9f}, '
            f'salto_máx={maximum_joint_steps[joint_index]:.9f}'
        )

    print()
    print('Distancia al centro de muñeca:')
    print(
        f'mín={wrist_distances.min():.9f} m, '
        f'máx={wrist_distances.max():.9f} m'
    )


if __name__ == '__main__':
    main()
