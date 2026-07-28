#!/usr/bin/env python3

"""
Nodo de ajuste PID para la trayectoria de lemniscata.

Adaptación a ROS 2 Jazzy y Gazebo Harmonic del archivo
tuning_node_de.py del repositorio We-R22/scorbot.

Se mantienen los parámetros y el flujo de ejecución originales.
"""

import sys
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import SwitchController
from ament_index_python.packages import get_package_share_directory

from pid_tuning.evolutive_algorithms.dif_evolution import (
    DifferentialEvolution,
)
from pid_tuning.evolutive_algorithms.evolutive_interface import Rate
from pid_tuning.settings.control_gazebo import ControlGazebo


# Parámetros originales de We-R22/scorbot.
A = 5
m = 15
N = 30
Gm = 50
F = 0.90
C = 0.90
hz = 25



def activate_tuning_controllers(
    node,
    reset_control,
) -> None:
    """
    Activa los controladores después de cargar las ganancias
    del primer individuo.
    """
    service_name = (
        '/controller_manager/switch_controller'
    )

    client = node.create_client(
        SwitchController,
        service_name,
    )

    try:
        node.get_logger().info(
            f'Esperando el servicio {service_name}...'
        )

        while (
            rclpy.ok()
            and not client.wait_for_service(timeout_sec=1.0)
        ):
            node.get_logger().info(
                'El servicio switch_controller '
                'todavía no está disponible.'
            )

        if not rclpy.ok():
            raise RuntimeError(
                'ROS 2 se cerró antes de encontrar '
                'switch_controller.'
            )

        request = SwitchController.Request()

        request.activate_controllers = [
            'joint_state_broadcaster',
            'scorbot_arm_controller',
        ]

        request.deactivate_controllers = []
        request.strictness = request.STRICT
        request.activate_asap = True
        request.timeout.sec = 10
        request.timeout.nanosec = 0

        node.get_logger().info(
            'Enviando solicitud de activación.'
        )

        future = client.call_async(request)

        # El controller_manager necesita que avance Gazebo
        # para completar el cambio de estado.
        reset_control.unpause()

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=15.0,
        )

        if not future.done():
            raise RuntimeError(
                'La activación no respondió dentro '
                'de 15 segundos.'
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                'switch_controller no devolvió respuesta.'
            )

        if not response.ok:
            raise RuntimeError(
                'No se pudieron activar los controladores: '
                f'{response.message}'
            )

        node.get_logger().info(
            'Controladores activados con las ganancias '
            'del individuo 0.'
        )

    finally:
        # evaluate() debe comenzar con el mundo pausado.
        reset_control.pause()
        node.destroy_client(client)


def evol_loop(arg1: str) -> None:
    """
    Ejecuta una optimización mediante Evolución Diferencial.

    Args:
        arg1:
            Identificador agregado al archivo de resultados.
    """
    rclpy.init(args=None)

    reset_control = None
    de = None

    try:
        scorbot_share = Path(
            get_package_share_directory('scorbot_gazebo')
        )

        paths_file = (
            scorbot_share
            / 'config'
            / 'paths.json'
        )

        if not paths_file.is_file():
            raise FileNotFoundError(
                f'No se encontró el archivo de configuración: '
                f'{paths_file}'
            )

        reset_control = ControlGazebo(
            world_name='empty',
        )

        de = DifferentialEvolution(
            N,
            m,
            Gm,
            F,
            C,
            A,
            str(paths_file),
            epsilon_1=10,
            tm=28800,
        )

        # Generación de población original.
        X = de.gen_population()

        output_path = (
            Path.cwd()
            / f'best_pid_values_DE_{arg1}.txt'
        )

        # tuning.launch.py inicia Gazebo pausado y deja
        # ambos controladores configurados como inactive.
        reset_control.init_values()

        de.node.get_logger().info(
            'Cargando las ganancias del individuo 0 '
            'antes de activar los controladores.'
        )

        de.node.get_logger().info(
            f'Ganancias X[0]: {X[0, :de.m]}'
        )

        # Se utiliza el mismo individuo completo que después
        # recibirá evaluate().
        de._update_pid_gains(
            X[0]
        )

        activate_tuning_controllers(
            de.node,
            reset_control,
        )

        # Equivalente ROS 2 de rospy.Rate(25).
        rate = Rate(
            de.node,
            hz,
        )

        with output_path.open(
            mode='w',
            encoding='utf-8',
        ) as file:

            while rclpy.ok():

                # Evaluación de la población inicial.
                de.evaluate(
                    X,
                    reset_control,
                    rate,
                )

                # Ejecución del algoritmo original.
                X_best = de.dif_evolution(
                    X,
                    reset_control,
                    rate,
                )

                file.write(str(X_best))
                file.write('\n')

                de.node.get_logger().info(
                    f'Resultado almacenado en: {output_path}'
                )

                break

    finally:
        if de is not None:
            de.destroy()

        if reset_control is not None:
            reset_control.destroy()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    if len(sys.argv) < 2:
        raise SystemExit(
            'Uso: tuning_node_de.py <identificador>'
        )

    arg1 = sys.argv[1]

    try:
        evol_loop(arg1)

    except KeyboardInterrupt:
        pass
