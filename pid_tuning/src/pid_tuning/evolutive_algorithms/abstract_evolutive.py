#!/usr/bin/env python3

"""
Clase abstracta de los algoritmos evolutivos.

Adaptación a ROS 2 Jazzy y Gazebo Harmonic del archivo original
de We-R22/pid_tuning.

Las funciones matemáticas bounds(), gen_population(), scv(), deb()
y deb_bubble_sort() conservan la implementación original.
"""

from .evolutive_interface import *


from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import TransformException


class AbstractEvolutive(EvolutiveInterface):

    def __init__(
        self,
        N: int,
        m: int,
        Gm: int,
        A: int,
        epsilon_1=0.10,
    ):
        EvolutiveInterface().__init__()

        self.N = N
        self.m = m
        self.Gm = Gm

        self.a = np.zeros((1, self.m))
        self.b = np.ones((1, self.m))

        self.A = A
        self.errors = np.zeros(self.A)

        self.dic_cmds = {}
        self.dic_sts = {}
        self.dic_cli = {}

        self.pubs = {}
        self.subs = {}

        self.g1_x = 0
        self.g1_y = 0
        self.g1_z = 0

        self.x_o = 0
        self.y_o = 0
        self.z_o = 0

        self.epsilon_1 = epsilon_1

        if not rclpy.ok():
            raise RuntimeError(
                'Debe ejecutarse rclpy.init() antes de crear '
                'DifferentialEvolution.'
            )

        self.node = Node('pid_tuning_evaluator')

        self.info = EvolutiveInfo()

        self.trajectories = None
        self.fb_trajectory = None

        self.joint_names = [
            f'j{i}'
            for i in range(1, self.A + 1)
        ]

        self.pid_parameter_names = {}

        self.reference_frame = 'base_link'
        self.end_effector_frame = 'tool_link'

        self.tf_buffer = None
        self.tf_listener = None

        self.initial_settle_steps = 50
        self.parameter_timeout_sec = 10.0

        self._tf_warning_printed = False

    def read_json(self, file_path: str):
        """
        Reads json file with paths, PID bounds and controller
        configuration.
        """
        with open(file_path, encoding='utf-8') as file:
            self.data = json.load(file)

    def _resolve_resource_path(self, resource_path: str) -> str:
        """
        Convierte una URI package:// en una ruta del sistema.

        Las rutas normales se devuelven sin modificaciones.
        """
        prefix = 'package://'

        if not resource_path.startswith(prefix):
            return resource_path

        resource = resource_path[len(prefix):]

        parts = resource.split('/', 1)

        if len(parts) != 2:
            raise ValueError(
                f'URI package:// no válida: {resource_path}'
            )

        package_name, relative_path = parts

        package_share = Path(
            get_package_share_directory(package_name)
        )

        return str(package_share / relative_path)

    def set_paths(self):
        """
        Creates dictionaries to store command and state topics and
        ROS 2 parameter clients.
        """
        cmd_paths = self.data['command_paths']
        sts_paths = self.data['state_paths']
        cl_paths = self.data['client_paths']

        for index, command_path in enumerate(cmd_paths, start=1):
            self.dic_cmds[f'command{index}'] = command_path

        for index, state_path in enumerate(sts_paths, start=1):
            self.dic_sts[f'state{index}'] = state_path

        for index, client_path in enumerate(cl_paths, start=1):
            self.dic_cli[f'client{index}'] = AsyncParameterClient(
                self.node,
                client_path,
            )

        self.joint_names = self.data.get(
            'joint_names',
            self.joint_names,
        )

        self.pid_parameter_names = self.data.get(
            'pid_parameter_names',
            {},
        )

        cartesian_frames = self.data.get(
            'cartesian_frames',
            {},
        )

        self.reference_frame = cartesian_frames.get(
            'reference',
            'base_link',
        )

        self.end_effector_frame = cartesian_frames.get(
            'end_effector',
            'tool_link',
        )

        evaluation_config = self.data.get('evaluation', {})

        self.initial_settle_steps = int(
            evaluation_config.get(
                'initial_settle_steps',
                50,
            )
        )

    def get_paths(self):
        """Returns command, state and parameter-client dictionaries."""
        return self.dic_cmds, self.dic_sts, self.dic_cli

    def set_pubssubs(self):
        """
        Initializes ROS 2 publishers, subscribers and TF listener.

        El Scorbot utiliza un solo JointTrajectoryController para
        sus cinco articulaciones.
        """
        for key, command_path in self.dic_cmds.items():
            index = key.replace('command', '')

            self.pubs[f'pub{index}'] = self.node.create_publisher(
                JointTrajectory,
                command_path,
                10,
            )

        for key, state_path in self.dic_sts.items():
            index = key.replace('state', '')

            self.subs[f'sub{index}'] = self.node.create_subscription(
                JointTrajectoryControllerState,
                state_path,
                self.error_callback,
                10,
            )

        self.pub = self.node.create_publisher(
            EvolutiveInfo,
            '/generations_info',
            10,
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.node,
            spin_thread=False,
        )

    def get_trajectories(self):
        """
        Reads CSV files containing the joint and Cartesian
        trajectories.
        """
        columns = self.data['trajectories']['column_names']

        trajectory_path = self._resolve_resource_path(
            self.data['trajectories']['path']
        )

        self.trajectories = pd.read_csv(
            trajectory_path,
            usecols=columns,
        )

        try:
            fb_columns = self.data[
                'trajectories'
            ]['fb_column_names']

            floating_base_path = self._resolve_resource_path(
                self.data[
                    'trajectories'
                ]['floating_base_path']
            )

            self.fb_trajectory = pd.read_csv(
                floating_base_path,
                usecols=fb_columns,
            )

            self.x_d = self.fb_trajectory['X']
            self.y_d = self.fb_trajectory['Y']
            self.z_d = self.fb_trajectory['Z']

        except Exception as exception:
            self.node.get_logger().warning(
                f'No se pudo cargar la trayectoria cartesiana: '
                f'{exception}'
            )

            size = len(self.trajectories)

            self.x_d = np.zeros(size)
            self.y_d = np.zeros(size)
            self.z_d = np.zeros(size)

    def bounds(self):
        """
        Establishes minimum and maximum limit values for each
        design variable.
        """
        pid_bounds = self.data['pid_bounds']

        p_min, p_max = pid_bounds['p']
        i_min, i_max = pid_bounds['i']
        d_min, d_max = pid_bounds['d']

        cnt = 0

        for i in range(self.m):
            if cnt == 0:
                self.a[0][i] += p_min
                self.b[0][i] = p_max * self.b[0][i]

            if cnt == 1:
                self.a[0][i] += i_min
                self.b[0][i] = i_max * self.b[0][i]

            if cnt == 2:
                self.a[0][i] += d_min
                self.b[0][i] = d_max * self.b[0][i]

            cnt = cnt + 1

            if cnt > 2:
                cnt = 0

    def gen_population(self):
        """
        Generates initial random population.
        """
        X = np.zeros((self.N, self.m + 2))

        self.bounds()

        for i in range(self.N):
            for j in range(self.m):
                X[i][j] = (
                    self.b[0][j] - self.a[0][j]
                ) * np.random.random_sample() - self.a[0][j]

        return X

    def error_callback(
        self,
        data: JointTrajectoryControllerState,
    ):
        """
        Accumulates the absolute tracking error for every joint.
        """
        state_errors = data.error.positions

        if len(state_errors) == 0:
            return

        if data.joint_names:
            for destination_index, joint_name in enumerate(
                self.joint_names
            ):
                try:
                    source_index = data.joint_names.index(
                        joint_name
                    )
                except ValueError:
                    continue

                if source_index < len(state_errors):
                    self.errors[destination_index] += abs(
                        float(state_errors[source_index])
                    )

        else:
            number_of_errors = min(
                self.A,
                len(state_errors),
            )

            for index in range(number_of_errors):
                self.errors[index] += abs(
                    float(state_errors[index])
                )

    def fb_callback(self, data):
        """
        Conserva el nombre del callback original.

        En ROS 2 la posición cartesiana se obtiene mediante TF entre
        base_link y tool_link, no mediante el tópico /odom.
        """
        if hasattr(data, 'transform'):
            self.x_o = data.transform.translation.x
            self.y_o = data.transform.translation.y
            self.z_o = data.transform.translation.z

        elif hasattr(data, 'pose'):
            self.x_o = data.pose.pose.position.x
            self.y_o = data.pose.pose.position.y
            self.z_o = data.pose.pose.position.z

    def _update_cartesian_position(self) -> bool:
        """Obtiene la posición del efector final usando TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.end_effector_frame,
                Time(),
            )

        except TransformException as exception:
            if not self._tf_warning_printed:
                self.node.get_logger().warning(
                    'No se pudo obtener la transformación '
                    f'{self.reference_frame} -> '
                    f'{self.end_effector_frame}: {exception}'
                )

                self._tf_warning_printed = True

            return False

        self.x_o = transform.transform.translation.x
        self.y_o = transform.transform.translation.y
        self.z_o = transform.transform.translation.z

        self._tf_warning_printed = False

        return True

    def _update_pid_gains(self, individual: np.ndarray) -> None:
        """
        Actualiza de manera atómica las 15 ganancias del
        JointTrajectoryController.
        """
        if 'client1' not in self.dic_cli:
            raise RuntimeError(
                'No existe el cliente de parámetros client1.'
            )

        client = self.dic_cli['client1']

        services_available = client.wait_for_services(
            timeout_sec=self.parameter_timeout_sec
        )

        if not services_available:
            raise RuntimeError(
                'No están disponibles los servicios de parámetros de '
                '/scorbot_arm_controller.'
            )

        parameters = []

        for joint_index, joint_name in enumerate(
            self.joint_names
        ):
            parameter_names = self.pid_parameter_names[
                joint_name
            ]

            parameters.extend(
                [
                    Parameter(
                        parameter_names['p'],
                        value=float(
                            individual[joint_index * 3]
                        ),
                    ),
                    Parameter(
                        parameter_names['i'],
                        value=float(
                            individual[joint_index * 3 + 1]
                        ),
                    ),
                    Parameter(
                        parameter_names['d'],
                        value=float(
                            individual[joint_index * 3 + 2]
                        ),
                    ),
                ]
            )

        future = client.set_parameters_atomically(parameters)

        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=self.parameter_timeout_sec,
        )

        if not future.done():
            raise TimeoutError(
                'El controlador no respondió al actualizar las '
                'ganancias PID.'
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                'El servicio de parámetros no devolvió respuesta.'
            )

        if not response.result.successful:
            raise RuntimeError(
                'No se pudieron actualizar las ganancias PID: '
                f'{response.result.reason}'
            )

    def _publish_positions(
        self,
        positions,
        period: float,
    ) -> None:
        """
        Publica una referencia para las cinco articulaciones.
        """
        if 'pub1' not in self.pubs:
            raise RuntimeError(
                'No existe el publicador de trayectoria pub1.'
            )

        message = JointTrajectory()

        message.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()

        point.positions = [
            float(position)
            for position in positions
        ]

        point.time_from_start = Duration(
            seconds=float(period)
        ).to_msg()

        message.points = [point]

        self.pubs['pub1'].publish(message)

    def _trajectory_positions(self, sample_index: int):
        """Obtiene las cinco referencias de una muestra del CSV."""
        return [
            float(
                self.trajectories[
                    f'q{joint_index}'
                ].iloc[sample_index]
            )
            for joint_index in range(1, self.A + 1)
        ]

    def _prepare_initial_pose(self, rate: Rate) -> None:
        """
        Lleva el robot a la primera postura de la trayectoria.

        Esta operación sustituye el reinicio completo del mundo que se
        utilizaba con Gazebo Classic.
        """
        initial_positions = self._trajectory_positions(0)

        for _ in range(self.initial_settle_steps):
            self._publish_positions(
                initial_positions,
                rate.period,
            )

            rate.sleep()

    def evaluate(
        self,
        P: np.ndarray,
        reset_control: ControlGazebo,
        rate: Rate,
    ):
        """
        Evaluates a population by updating PID gains and sending the
        reference joint trajectory.
        """
        for w, individual in zip(range(self.N), P):

            reset_control.pause()

            self._update_pid_gains(individual)

            reset_control.unpause()

            self._prepare_initial_pose(rate)

            self.errors[:] = 0.0

            self.g1_x = 0.0
            self.g1_y = 0.0
            self.g1_z = 0.0

            length = len(self.trajectories)

            for sample_index in range(length):

                positions = self._trajectory_positions(
                    sample_index
                )

                self._publish_positions(
                    positions,
                    rate.period,
                )

                rate.sleep()

                self._update_cartesian_position()

                self.g1_x += abs(
                    float(self.x_d.iloc[sample_index])
                    - self.x_o
                )

                self.g1_y += abs(
                    float(self.y_d.iloc[sample_index])
                    - self.y_o
                )

                self.g1_z += abs(
                    float(self.z_d.iloc[sample_index])
                    - self.z_o
                )

            reset_control.pause()

            P[w][self.m] = sum(self.errors)

            P[w][self.m + 1] = 0.0
            P[w][self.m + 1] = self.scv()

            self.info.header.stamp = (
                self.node.get_clock().now().to_msg()
            )

            self.info.header.frame_id = self.reference_frame

            self.info.individual = w

            self.info.genes = [
                float(value)
                for value in P[w, 0:-2]
            ]

            self.info.of = float(P[w][-2])
            self.info.scv = float(P[w][-1])

            self.pub.publish(self.info)

            self.node.get_logger().info(
                f'Individuo {w}: '
                f'OF={P[w][-2]:.6f}, '
                f'SCV={P[w][-1]:.6f}'
            )

    def scv(self):
        """
        Penality function used to obtain the sum of constraint
        violation.
        """
        self.f = 0
        vvr = 0

        g1 = self.g1_x + self.g1_y + self.g1_z

        vvr = abs(g1) - self.epsilon_1

        if vvr > 0:
            self.f = self.f + vvr

        return self.f

    def deb(self, u: np.ndarray, x: np.ndarray):
        """
        Uses feasible Deb rules as selection criteria.
        """
        if u[-1] <= 0 and x[-1] > 0:
            return 1

        if u[-1] > 0 and x[-1] > 0:
            if u[-1] <= x[-1]:
                return 1

        else:
            if u[-1] <= 0 and x[-1] <= 0:
                if u[-2] < x[-2]:
                    return 1

        return 0

    def deb_bubble_sort(self, P: np.ndarray):
        """
        Sorts the population using BubbleSort and Deb's rules.
        """
        n = len(P)

        for i in range(n - 1):
            for j in range(n - 1):

                x = np.array(
                    [
                        P[j][-2],
                        P[j][-1],
                    ]
                )

                u = np.array(
                    [
                        P[j + 1][-2],
                        P[j + 1][-1],
                    ]
                )

                if self.deb(u, x) == 1:
                    copy = P[j, :].copy()

                    P[j, :] = P[j + 1, :]
                    P[j + 1, :] = copy

        return P

    def destroy(self) -> None:
        """Libera el nodo interno utilizado por el evaluador."""
        if self.node is not None:
            self.node.destroy_node()
