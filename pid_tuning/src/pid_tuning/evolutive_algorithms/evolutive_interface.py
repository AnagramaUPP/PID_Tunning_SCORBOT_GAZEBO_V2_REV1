#!/usr/bin/env python3

"""
Interfaz común de los métodos evolutivos.

Adaptación a ROS 2 Jazzy del archivo original perteneciente a
We-R22/pid_tuning.

Se conservan la clase EvolutiveInterface y los nombres originales
de sus métodos.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from random import choice, randint, random

import numpy as np
import pandas as pd
import rclpy

from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pid_tuning.msg import EvolutiveInfo
from pid_tuning.settings.control_gazebo import ControlGazebo


class Rate:
    """
    Equivalente en ROS 2 de rospy.Rate.

    Además de respetar la frecuencia indicada, procesa callbacks del
    nodo durante la espera. Esto permitirá recibir el estado del
    controlador y las transformaciones TF mientras se publica la
    trayectoria.
    """

    def __init__(self, node: Node, hz: float) -> None:
        if hz <= 0:
            raise ValueError('La frecuencia debe ser mayor que cero.')

        self.node = node
        self.hz = float(hz)
        self.period = 1.0 / self.hz
        self.next_time = time.monotonic()

    def sleep(self) -> None:
        """Espera hasta el siguiente periodo procesando callbacks ROS 2."""

        self.next_time += self.period

        while rclpy.ok():
            remaining = self.next_time - time.monotonic()

            if remaining <= 0.0:
                # Si el proceso se retrasó más de un periodo,
                # reinicia la referencia temporal.
                if remaining < -self.period:
                    self.next_time = time.monotonic()

                return

            rclpy.spin_once(
                self.node,
                timeout_sec=min(remaining, 0.01),
            )


class EvolutiveInterface:
    def __init__(self) -> None:
        pass

    def gen_population(self):
        pass

    def read_json(self):
        pass

    def bounds(self):
        pass

    def error_callback(self):
        pass

    def evaluate(self):
        pass

    def set_paths(self):
        pass

    def get_paths(self):
        pass

    def set_pubssubs(self):
        pass

    def get_trajectories(self):
        pass

    def scv(self):
        pass

    def fb_callback(self):
        pass

    def deb(self):
        pass
