#!/usr/bin/env python3

"""
Control del mundo de Gazebo Harmonic mediante ROS 2.

Adaptación del archivo control_gazebo.py del repositorio
We-R22/pid_tuning.

Se conserva la clase ControlGazebo y su interfaz principal,
pero los servicios de Gazebo Classic se sustituyen por
ros_gz_interfaces/srv/ControlWorld.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld


class ControlGazebo:
    """Controla la pausa y reanudación de un mundo de Gazebo Harmonic."""

    def __init__(
        self,
        world_name: str = 'empty',
        service_timeout_sec: float = 10.0,
    ) -> None:
        """
        Inicializa el cliente del servicio de control del mundo.

        Args:
            world_name:
                Nombre del mundo ejecutado en Gazebo Harmonic.
            service_timeout_sec:
                Tiempo máximo para localizar el servicio y esperar
                cada respuesta.
        """
        if not rclpy.ok():
            raise RuntimeError(
                'rclpy.init() debe ejecutarse antes de crear ControlGazebo.'
            )

        self._world_name = world_name
        self._service_timeout_sec = float(service_timeout_sec)
        self._service_name = f'/world/{world_name}/control'

        self._node: Node = rclpy.create_node('control_gazebo')

        self._control_client = self._node.create_client(
            ControlWorld,
            self._service_name,
        )

        self._node.get_logger().info(
            f'Esperando el servicio {self._service_name}...'
        )

        service_available = self._control_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        )

        if not service_available:
            self._node.destroy_node()
            raise RuntimeError(
                f'No se encontró el servicio {self._service_name}. '
                'Comprueba que Gazebo Harmonic y el puente del servicio '
                'estén activos.'
            )

        self._node.get_logger().info(
            f'Servicio encontrado: {self._service_name}'
        )

    @property
    def node(self) -> Node:
        """Devuelve el nodo ROS 2 interno."""
        return self._node

    @property
    def service_name(self) -> str:
        """Devuelve el nombre del servicio utilizado."""
        return self._service_name

    def _send_pause_command(self, pause: bool) -> bool:
        """
        Envía una solicitud para pausar o reanudar el mundo.

        Args:
            pause:
                True para pausar y False para reanudar.

        Returns:
            True cuando Gazebo acepta la solicitud.

        Raises:
            TimeoutError:
                Cuando Gazebo no responde dentro del tiempo configurado.
            RuntimeError:
                Cuando el servicio devuelve un error.
        """
        request = ControlWorld.Request()
        request.world_control.pause = bool(pause)

        future = self._control_client.call_async(request)

        rclpy.spin_until_future_complete(
            self._node,
            future,
            timeout_sec=self._service_timeout_sec,
        )

        if not future.done():
            raise TimeoutError(
                f'Gazebo no respondió al servicio {self._service_name} '
                f'en {self._service_timeout_sec:.1f} segundos.'
            )

        try:
            response = future.result()
        except Exception as exc:
            raise RuntimeError(
                f'Falló la llamada a {self._service_name}: {exc}'
            ) from exc

        if response is None:
            raise RuntimeError(
                f'El servicio {self._service_name} no devolvió respuesta.'
            )

        if not response.success:
            action = 'pausar' if pause else 'reanudar'
            raise RuntimeError(
                f'Gazebo rechazó la solicitud para {action} el mundo.'
            )

        return True

    def init_values(self) -> bool:
        """
        Prepara el mundo antes de una evaluación.

        En Gazebo Classic, el método original reiniciaba toda la
        simulación y modificaba las propiedades físicas.

        En Gazebo Harmonic no se ejecuta reset.all porque ese reinicio
        elimina y reconstruye el modelo y sus controladores. La postura
        inicial del robot se preparará posteriormente desde la capa de
        evaluación y ros2_control.
        """
        self._node.get_logger().info(
            'Preparando Gazebo Harmonic: el mundo quedará pausado.'
        )
        return self.pause()

    def update(self) -> bool:
        """
        Reanuda la simulación.

        Las propiedades físicas del mundo se establecen en el archivo
        SDF de Gazebo Harmonic y no mediante SetPhysicsProperties.
        """
        return self.unpause()

    def pause(self) -> bool:
        """Pausa el mundo de Gazebo Harmonic."""
        result = self._send_pause_command(True)

        self._node.get_logger().debug(
            f'Mundo {self._world_name} pausado.'
        )

        return result

    def unpause(self) -> bool:
        """Reanuda el mundo de Gazebo Harmonic."""
        result = self._send_pause_command(False)

        self._node.get_logger().debug(
            f'Mundo {self._world_name} reanudado.'
        )

        return result

    def destroy(self) -> None:
        """Destruye el nodo ROS 2 interno."""
        if self._node is not None:
            self._node.destroy_node()
