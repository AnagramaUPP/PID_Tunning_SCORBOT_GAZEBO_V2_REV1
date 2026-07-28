#!/usr/bin/env python3

"""
Recicla rápidamente scorbot_arm_controller cuando el plugin
de Gazebo informa el inicio de un nuevo individuo.

No modifica el algoritmo evolutivo, no reinicia el mundo
y no elimina ni vuelve a insertar el robot.
"""

from enum import Enum, auto
import time

import rclpy
from controller_manager_msgs.srv import (
    ListControllers,
    SwitchController,
)
from rclpy.node import Node
from std_msgs.msg import Int32


class SupervisorState(Enum):
    IDLE = auto()
    WAITING_LIST = auto()
    WAITING_DEACTIVATE = auto()
    WAITING_ACTIVATE = auto()


class ScorbotPidResetSupervisor(Node):
    """Limpia la memoria PID entre individuos."""

    def __init__(self) -> None:
        super().__init__('scorbot_pid_reset_supervisor')

        self.controller_name = 'scorbot_arm_controller'

        self.list_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )

        self.switch_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
        )

        self.create_subscription(
            Int32,
            '/scorbot/home_reset',
            self.home_reset_callback,
            10,
        )

        self.reset_done_publisher = self.create_publisher(
            Int32,
            '/scorbot/pid_reset_done',
            10,
        )

        # Tiempo de pared: funciona aunque el reloj de
        # simulación se encuentre pausado.
        self.timer = self.create_timer(
            0.002,
            self.process_state_machine,
        )

        self.state = SupervisorState.IDLE
        self.pending_reset = False
        self.future = None
        self.deadline = 0.0
        self.event_start_time = 0.0
        self.current_reset_id = 0
        self.completed_resets = 0

        self.get_logger().info(
            'Supervisor PID iniciado. Esperando eventos en '
            '/scorbot/home_reset.'
        )

    def home_reset_callback(self, message: Int32) -> None:
        """Recibe el inicio inequívoco de un individuo."""

        reset_id = int(message.data)

        if reset_id <= 1:
            return

        if (
            self.pending_reset
            or self.state is not SupervisorState.IDLE
        ):
            self.get_logger().warning(
                f'Evento #{reset_id} recibido mientras el '
                'reciclado anterior continúa.'
            )
            return

        self.current_reset_id = reset_id
        self.pending_reset = True
        self.event_start_time = time.monotonic()

        self.get_logger().info(
            f'Inicio de individuo detectado mediante '
            f'reposicionamiento #{reset_id}.'
        )

    def process_state_machine(self) -> None:
        """Ejecuta el reciclado sin bloquear el nodo."""

        now = time.monotonic()

        if self.state is SupervisorState.IDLE:
            if not self.pending_reset:
                return

            if not self.list_client.service_is_ready():
                return

            self.future = self.list_client.call_async(
                ListControllers.Request()
            )

            self.deadline = now + 2.0
            self.state = SupervisorState.WAITING_LIST
            return

        if now > self.deadline:
            self.fail_operation(
                'Tiempo agotado esperando al controller_manager.'
            )
            return

        if self.future is None or not self.future.done():
            return

        if self.state is SupervisorState.WAITING_LIST:
            self.process_controller_list()
        elif self.state is SupervisorState.WAITING_DEACTIVATE:
            self.process_deactivation()
        elif self.state is SupervisorState.WAITING_ACTIVATE:
            self.process_activation()

    def process_controller_list(self) -> None:
        """Verifica que el controlador esté activo."""

        try:
            response = self.future.result()
        except Exception as exception:
            self.fail_operation(
                f'Falló list_controllers: {exception}'
            )
            return

        controller_state = None

        for controller in response.controller:
            if controller.name == self.controller_name:
                controller_state = controller.state
                break

        if controller_state is None:
            self.fail_operation(
                'No se encontró scorbot_arm_controller.'
            )
            return

        if controller_state != 'active':
            self.fail_operation(
                'Se recibió un inicio de individuo, pero '
                f'el controlador está {controller_state}.'
            )
            return

        self.get_logger().info(
            'Controlador activo; comenzando reciclado PID.'
        )

        self.send_switch_request(
            activate=[],
            deactivate=[self.controller_name],
            next_state=SupervisorState.WAITING_DEACTIVATE,
        )

    def send_switch_request(
        self,
        activate,
        deactivate,
        next_state: SupervisorState,
    ) -> None:
        """Solicita una transición al controller_manager."""

        if not self.switch_client.service_is_ready():
            self.fail_operation(
                'switch_controller no está disponible.'
            )
            return

        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = request.STRICT
        request.activate_asap = True
        request.timeout.sec = 2
        request.timeout.nanosec = 0

        self.future = self.switch_client.call_async(request)
        self.deadline = time.monotonic() + 2.0
        self.state = next_state

    def process_deactivation(self) -> None:
        """Reactiva el controlador después de desactivarlo."""

        try:
            response = self.future.result()
        except Exception as exception:
            self.fail_operation(
                f'Falló la desactivación: {exception}'
            )
            return

        if response is None or not response.ok:
            message = (
                response.message
                if response is not None
                else 'sin respuesta'
            )

            self.fail_operation(
                f'No se desactivó el controlador: {message}'
            )
            return

        self.get_logger().info(
            'Controlador desactivado; solicitando reactivación.'
        )

        self.send_switch_request(
            activate=[self.controller_name],
            deactivate=[],
            next_state=SupervisorState.WAITING_ACTIVATE,
        )

    def process_activation(self) -> None:
        """Finaliza el reciclado."""

        try:
            response = self.future.result()
        except Exception as exception:
            self.fail_operation(
                f'Falló la activación: {exception}'
            )
            return

        if response is None or not response.ok:
            message = (
                response.message
                if response is not None
                else 'sin respuesta'
            )

            self.fail_operation(
                f'No se reactivó el controlador: {message}'
            )
            return

        self.completed_resets += 1

        confirmation = Int32()
        confirmation.data = self.current_reset_id

        self.reset_done_publisher.publish(
            confirmation
        )

        elapsed_ms = (
            time.monotonic() - self.event_start_time
        ) * 1000.0

        self.get_logger().info(
            f'Reciclado PID del reposicionamiento '
            f'#{self.current_reset_id} completado en '
            f'{elapsed_ms:.3f} ms. Confirmación publicada.'
        )

        self.clear_operation()

    def fail_operation(self, message: str) -> None:
        """Registra el fallo y libera la máquina de estados."""

        self.get_logger().error(message)
        self.clear_operation()

    def clear_operation(self) -> None:
        self.state = SupervisorState.IDLE
        self.pending_reset = False
        self.future = None
        self.deadline = 0.0
        self.current_reset_id = 0


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ScorbotPidResetSupervisor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
