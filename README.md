# PID Tuning para Scorbot en ROS 2 Jazzy y Gazebo Harmonic

Sistema de ajuste automático de ganancias PID para un manipulador Scorbot de cinco articulaciones mediante **Evolución Diferencial (DE)**, **ROS 2 Jazzy**, **Gazebo Harmonic** y `ros2_control`.

El repositorio contiene dos paquetes ROS 2:

```text
PID_Tunning_SCORBOT_GAZEBO_V2_REV1/
├── pid_tuning/
└── scorbot_gazebo/
```

- `pid_tuning`: núcleo del algoritmo evolutivo.
- `scorbot_gazebo`: modelo, controladores, simulación, trayectorias, reinicio rápido y scripts de ejecución.

## Objetivo

Encontrar automáticamente las ganancias PID de las articulaciones:

```text
j1, j2, j3, j4 y j5
```

Cada individuo contiene 15 variables:

```text
Kp1, Ki1, Kd1,
Kp2, Ki2, Kd2,
Kp3, Ki3, Kd3,
Kp4, Ki4, Kd4,
Kp5, Ki5, Kd5
```

## Características

- Gazebo Harmonic con gravedad activa.
- `JointTrajectoryController` por esfuerzo.
- Actualización dinámica de ganancias PID.
- Evaluación mediante trayectoria articular.
- Cálculo de OF y SCV.
- Reinicio de posición y velocidad entre individuos.
- Reinicio de la memoria interna del PID.
- Sin `reset_world`.
- Sin eliminar ni volver a insertar el robot.
- Sin reutilizar el estado final del individuo anterior.
- Ejecución de una corrida o 30 corridas independientes.
- Prueba de ganancias fijas.
- Registro de resultados y mejores ganancias.

## Entorno validado

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3
- `ros_gz_sim`
- `ros_gz_bridge`
- `gz_ros2_control`
- `ros2_control`
- `ros2_controllers`

## Estructura

```text
PID_Tunning_SCORBOT_GAZEBO_V2_REV1/
├── pid_tuning/
│   ├── msg/
│   ├── src/pid_tuning/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── setup.py
├── scorbot_gazebo/
│   ├── config/
│   ├── launch/
│   ├── meshes/
│   ├── scripts/
│   ├── src/
│   ├── trajectories/
│   ├── urdf/
│   ├── worlds/
│   ├── CMakeLists.txt
│   └── package.xml
├── docs/
│   ├── ARCHITECTURE.md
│   ├── USAGE.md
│   └── VALIDATION.md
└── README.md
```

## Clonar

```bash
cd ~/ros2_ws/src

git clone \
    https://github.com/AnagramaUPP/PID_Tunning_SCORBOT_GAZEBO_V2_REV1.git
```

## Compilar

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build \
    --packages-select pid_tuning scorbot_gazebo \
    --symlink-install

source install/setup.bash
```

## Ejecutar una optimización

```bash
ros2 run \
    scorbot_gazebo \
    run_tuning_de.sh \
    corrida_01
```

## Ejecutar 30 corridas independientes

```bash
ros2 run \
    scorbot_gazebo \
    run_30.sh \
    corridas_30
```

El último argumento solo identifica el lote. `run_30.sh` ejecuta 30 optimizaciones completas, no 30 individuos.

## Probar ganancias fijas

Coloque las 15 ganancias en:

```text
scorbot_gazebo/scripts/test_fixed_pid.py
```

Luego:

```bash
ros2 run \
    scorbot_gazebo \
    run_test_pid.sh \
    mejor_pid
```

## Límites PID

Archivo:

```text
scorbot_gazebo/config/paths.json
```

Configuración actual:

```json
"pid_bounds": {
    "p": [0, 250],
    "i": [0, 10],
    "d": [0, 1]
}
```

## Trayectoria

Archivo principal:

```text
scorbot_gazebo/trajectories/joints_trajectory.csv
```

Columnas:

```text
q1, q2, q3, q4, q5
```

La primera fila define la postura inicial de cada evaluación.

## Reinicio entre individuos

Cada individuo sigue este flujo:

1. pausar Gazebo;
2. actualizar ganancias;
3. fijar posición inicial;
4. fijar velocidades en cero;
5. reciclar el controlador;
6. confirmar reinicio del PID;
7. liberar el robot;
8. ejecutar la trayectoria;
9. calcular OF y SCV.

## Resultados

Normalmente se guardan en:

```text
~/ros2_ws/results/
```

Archivos típicos:

```text
best_pid_values_DE_<identificador>.txt
<identificador>_de.log
<identificador>_gazebo.log
```

Interpretación:

- `OF`: función objetivo.
- `SCV`: suma de violaciones de restricciones.
- `SCV = 0`: sin violaciones detectadas.
- código de salida `0`: ejecución correcta.


## Limitaciones

Esta revisión está configurada específicamente para:

- Scorbot de cinco articulaciones;
- articulaciones `j1` a `j5`;
- controlador `scorbot_arm_controller`;
- PID articular por esfuerzo;
- trayectoria de cinco columnas.

## Recomendaciones

- No cambiar física, trayectoria o límites durante una campaña.
- No ejecutar dos Gazebo al mismo tiempo.
- Registrar el commit utilizado.
- Conservar los logs.
- No subir `build/`, `install/`, `log/` ni `results/`.

## Licencia

Consulte los archivos `LICENSE` de los paquetes.

## Créditos

Repositorio mantenido por **AnagramaUPP**.
