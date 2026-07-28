#!/usr/bin/env python3

"""
Ventana 3D para visualizar el seguimiento de la lemniscata.

Rojo:
    Trayectoria cartesiana deseada.

Azul:
    Trayectoria real obtenida mediante TF:
    base_link -> tool_link.

La trayectoria real se limpia al comenzar cada individuo.
"""

from pathlib import Path
from threading import Lock, Thread

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rclpy
from ament_index_python.packages import (
    get_package_share_directory,
)
from pid_tuning.msg import EvolutiveInfo
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory


class CartesianTrackingPlotter(Node):

    def __init__(self) -> None:
        super().__init__('cartesian_tracking_plotter')

        self.reference_frame = 'base_link'
        self.end_effector_frame = 'tool_link'

        self.data_lock = Lock()

        self.recording = False
        self.waiting_for_start = True
        self.evaluation_number = 0

        self.actual_points = []

        self.generation = None
        self.individual = None
        self.objective_function = None
        self.scv = None

        self.rmse_x = None
        self.rmse_y = None
        self.rmse_z = None
        self.rmse_total = None

        self.status = (
            'Esperando el inicio de la evaluación'
        )

        self._load_trajectories()

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.create_subscription(
            JointTrajectory,
            '/scorbot_arm_controller/joint_trajectory',
            self.command_callback,
            20,
        )

        self.create_subscription(
            EvolutiveInfo,
            '/generations_info',
            self.evolutive_info_callback,
            20,
        )

        # Misma frecuencia utilizada por el evaluador.
        self.create_timer(
            1.0 / 25.0,
            self.sample_real_position,
        )

        self.get_logger().info(
            'Ventana gráfica de seguimiento cartesiano iniciada.'
        )

    def _load_trajectories(self) -> None:
        package_share = Path(
            get_package_share_directory('scorbot_gazebo')
        )

        cartesian_path = (
            package_share
            / 'trajectories'
            / 'cartesian_trajectory.csv'
        )

        joints_path = (
            package_share
            / 'trajectories'
            / 'joints_trajectory.csv'
        )

        self.desired_trajectory = np.loadtxt(
            cartesian_path,
            delimiter=',',
            skiprows=1,
            usecols=(0, 1, 2),
            dtype=np.float64,
        )

        joint_trajectory = np.loadtxt(
            joints_path,
            delimiter=',',
            skiprows=1,
            usecols=(0, 1, 2, 3, 4),
            dtype=np.float64,
        )

        if self.desired_trajectory.shape != (200, 3):
            raise RuntimeError(
                'Forma cartesiana incorrecta: '
                f'{self.desired_trajectory.shape}'
            )

        if joint_trajectory.shape != (200, 5):
            raise RuntimeError(
                'Forma articular incorrecta: '
                f'{joint_trajectory.shape}'
            )

        self.initial_joint_pose = (
            joint_trajectory[0].copy()
        )

    def command_callback(
        self,
        message: JointTrajectory,
    ) -> None:
        if not message.points:
            return

        positions = np.asarray(
            message.points[-1].positions,
            dtype=np.float64,
        )

        if positions.size < 5:
            return

        positions = positions[:5]

        at_initial_pose = np.allclose(
            positions,
            self.initial_joint_pose,
            atol=1e-5,
            rtol=0.0,
        )

        with self.data_lock:

            # _prepare_initial_pose envía q0 antes de iniciar.
            # La evaluación comienza cuando la referencia
            # abandona q0.
            if (
                self.waiting_for_start
                and not at_initial_pose
            ):
                self.evaluation_number += 1

                # Limpiar la trayectoria real anterior.
                self.actual_points = []

                self.recording = True
                self.waiting_for_start = False

                self.generation = None
                self.individual = None
                self.objective_function = None
                self.scv = None

                self.rmse_x = None
                self.rmse_y = None
                self.rmse_z = None
                self.rmse_total = None

                self.status = (
                    f'Evaluación '
                    f'{self.evaluation_number} en curso'
                )

                self.get_logger().info(
                    self.status
                )

    def sample_real_position(self) -> None:
        with self.data_lock:
            recording = self.recording

        if not recording:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
            )

        except Exception:
            return

        point = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]

        with self.data_lock:
            if self.recording:
                self.actual_points.append(point)

    def _calculate_rmse_locked(self) -> None:
        actual = np.asarray(
            self.actual_points,
            dtype=np.float64,
        )

        if actual.ndim != 2 or actual.shape[0] < 2:
            return

        desired = self.desired_trajectory

        # Si el número de muestras reales no es exactamente
        # 200, se interpola respecto al avance normalizado.
        actual_progress = np.linspace(
            0.0,
            1.0,
            actual.shape[0],
        )

        desired_progress = np.linspace(
            0.0,
            1.0,
            desired.shape[0],
        )

        actual_resampled = np.column_stack([
            np.interp(
                desired_progress,
                actual_progress,
                actual[:, axis],
            )
            for axis in range(3)
        ])

        error = desired - actual_resampled

        rmse_axes = np.sqrt(
            np.mean(
                error**2,
                axis=0,
            )
        )

        self.rmse_x = float(rmse_axes[0])
        self.rmse_y = float(rmse_axes[1])
        self.rmse_z = float(rmse_axes[2])

        self.rmse_total = float(
            np.sqrt(
                np.mean(
                    np.sum(
                        error**2,
                        axis=1,
                    )
                )
            )
        )

    def evolutive_info_callback(
        self,
        message: EvolutiveInfo,
    ) -> None:
        with self.data_lock:
            self.recording = False
            self.waiting_for_start = True

            self.generation = int(
                message.generation
            )

            self.individual = int(
                message.individual
            )

            self.objective_function = float(
                message.of
            )

            self.scv = float(
                message.scv
            )

            self._calculate_rmse_locked()

            self.status = (
                f'Generación {self.generation} | '
                f'Individuo {self.individual}'
            )

        self.get_logger().info(
            'Trayectoria terminada: '
            f'{self.status}'
        )

    def snapshot(self):
        with self.data_lock:
            actual = np.asarray(
                self.actual_points,
                dtype=np.float64,
            ).copy()

            information = {
                'status': self.status,
                'generation': self.generation,
                'individual': self.individual,
                'of': self.objective_function,
                'scv': self.scv,
                'rmse_x': self.rmse_x,
                'rmse_y': self.rmse_y,
                'rmse_z': self.rmse_z,
                'rmse_total': self.rmse_total,
                'samples': len(self.actual_points),
            }

        return actual, information


def configure_equal_axes(
    axis,
    trajectory: np.ndarray,
) -> None:
    minimum = trajectory.min(axis=0)
    maximum = trajectory.max(axis=0)

    center = (
        minimum + maximum
    ) / 2.0

    largest_span = float(
        np.max(
            maximum - minimum
        )
    )

    half_span = max(
        largest_span * 0.65,
        0.08,
    )

    axis.set_xlim(
        center[0] - half_span,
        center[0] + half_span,
    )

    axis.set_ylim(
        center[1] - half_span,
        center[1] + half_span,
    )

    axis.set_zlim(
        center[2] - half_span,
        center[2] + half_span,
    )

    axis.set_box_aspect((1, 1, 1))


def create_plot(node: CartesianTrackingPlotter):
    figure = plt.figure(
        figsize=(10, 8)
    )

    axis = figure.add_subplot(
        111,
        projection='3d',
    )

    desired = node.desired_trajectory

    axis.plot(
        desired[:, 0],
        desired[:, 1],
        desired[:, 2],
        color='red',
        linewidth=2.0,
        label='Referencia',
    )

    real_line, = axis.plot(
        [],
        [],
        [],
        color='blue',
        linewidth=2.0,
        label='Real',
    )

    axis.set_xlabel('X [m]')
    axis.set_ylabel('Y [m]')
    axis.set_zlabel('Z [m]')

    axis.legend(
        loc='upper right'
    )

    configure_equal_axes(
        axis,
        desired,
    )

    title = axis.set_title(
        'Esperando el inicio de la evaluación'
    )

    information_text = figure.text(
        0.02,
        0.02,
        'Muestras reales: 0',
        fontsize=10,
    )

    figure.tight_layout(
        rect=[0.0, 0.08, 1.0, 1.0]
    )

    def update_plot(_frame):
        actual, information = node.snapshot()

        if (
            actual.ndim == 2
            and actual.shape[0] > 0
        ):
            real_line.set_data(
                actual[:, 0],
                actual[:, 1],
            )

            real_line.set_3d_properties(
                actual[:, 2]
            )

        else:
            real_line.set_data([], [])
            real_line.set_3d_properties([])

        title.set_text(
            information['status']
        )

        lines = [
            f"Muestras reales: "
            f"{information['samples']}"
        ]

        if information['of'] is not None:
            lines.append(
                f"OF: {information['of']:.6f}    "
                f"SCV: {information['scv']:.6f}"
            )

        if information['rmse_total'] is not None:
            lines.append(
                f"RMSE total: "
                f"{information['rmse_total']:.6f} m"
            )

            lines.append(
                f"RMSE X: {information['rmse_x']:.6f} m    "
                f"Y: {information['rmse_y']:.6f} m    "
                f"Z: {information['rmse_z']:.6f} m"
            )

        information_text.set_text(
            '\n'.join(lines)
        )

        return (
            real_line,
            title,
            information_text,
        )

    animation = FuncAnimation(
        figure,
        update_plot,
        interval=100,
        blit=False,
        cache_frame_data=False,
    )

    return figure, animation


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CartesianTrackingPlotter()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    executor_thread = Thread(
        target=executor.spin,
        daemon=True,
    )

    executor_thread.start()

    figure = None

    try:
        figure, animation = create_plot(node)

        # Mantener la referencia para evitar que la animación
        # sea eliminada por el recolector de memoria.
        figure._cartesian_tracking_animation = animation

        plt.show()

    except KeyboardInterrupt:
        pass

    finally:
        if figure is not None:
            plt.close(figure)

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        executor_thread.join(
            timeout=2.0
        )


if __name__ == '__main__':
    main()
