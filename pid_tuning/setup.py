#!/usr/bin/env python3

from setuptools import find_packages, setup


package_name = 'pid_tuning'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Mitchell Angel Gomez Ortega',
    maintainer_email='mitchellgomez@upp.edu.mx',
    description=(
        'Adaptación a ROS 2 del paquete We-R22/pid_tuning '
        'para ajuste PID mediante Evolución Diferencial.'
    ),
    license='BSD-3-Clause',
)
