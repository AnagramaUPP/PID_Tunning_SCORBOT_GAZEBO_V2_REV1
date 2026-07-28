#!/usr/bin/env bash

set -o pipefail

WORKSPACE="${HOME}/ros2_ws"
PACKAGE="scorbot_gazebo"
LAUNCH_FILE="tuning.launch.py"
DE_EXECUTABLE="tuning_node_de.py"

RUN_NAME="${1:-evolucion_diferencial}"

if [[ ! "${RUN_NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "ERROR: el identificador solo puede contener:"
    echo "letras, números, guion y guion bajo."
    exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_ID="${RUN_NAME}_${STAMP}"

RESULTS_DIRECTORY="${WORKSPACE}/results/tuning_DE_${RUN_ID}"

mkdir -p "${RESULTS_DIRECTORY}"

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"

cd "${RESULTS_DIRECTORY}" || exit 1

LAUNCH_LOG="${RESULTS_DIRECTORY}/${RUN_ID}_gazebo.log"
DE_LOG="${RESULTS_DIRECTORY}/${RUN_ID}_de.log"

LAUNCH_PID=""
DE_STATUS=1


pause_world()
{
    timeout 3 \
        ros2 service call \
        /world/empty/control \
        ros_gz_interfaces/srv/ControlWorld \
        "{world_control: {pause: true}}" \
        > /dev/null 2>&1 \
        || true
}


stop_simulation()
{
    if [[ -z "${LAUNCH_PID}" ]]; then
        return
    fi

    echo
    echo "Pausando y cerrando Gazebo..."

    pause_world

    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true

    for _ in $(seq 1 20); do
        if ! kill -0 -- "-${LAUNCH_PID}" 2>/dev/null; then
            break
        fi

        sleep 1
    done

    if kill -0 -- "-${LAUNCH_PID}" 2>/dev/null; then
        kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
        sleep 2
    fi

    wait "${LAUNCH_PID}" 2>/dev/null || true

    LAUNCH_PID=""
}


wait_for_controllers()
{
    local controllers
    local arm_ready
    local broadcaster_ready

    for _ in $(seq 1 120); do

        if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            echo
            echo "ERROR: el launch terminó antes de preparar"
            echo "los controladores."

            tail -n 80 "${LAUNCH_LOG}"

            return 1
        fi

        controllers="$(
            timeout 3 \
                ros2 control list_controllers \
                2>/dev/null \
                || true
        )"

        arm_ready="$(
            printf '%s\n' "${controllers}" |
            grep -E \
                '^scorbot_arm_controller[[:space:]].*[[:space:]]inactive$' \
                || true
        )"

        broadcaster_ready="$(
            printf '%s\n' "${controllers}" |
            grep -E \
                '^joint_state_broadcaster[[:space:]].*[[:space:]]inactive$' \
                || true
        )"

        if [[
            -n "${arm_ready}"
            && -n "${broadcaster_ready}"
        ]]; then
            return 0
        fi

        sleep 1
    done

    echo
    echo "ERROR: tiempo agotado esperando los controladores."

    tail -n 80 "${LAUNCH_LOG}"

    return 1
}


verify_configuration()
{
    python3 - "${WORKSPACE}" <<'PY'
from pathlib import Path
import json
import sys

workspace = Path(sys.argv[1])

paths = [
    workspace / 'src/scorbot_gazebo/config/paths.json',
    workspace / (
        'install/scorbot_gazebo/share/'
        'scorbot_gazebo/config/paths.json'
    ),
]

for path in paths:
    if not path.exists():
        raise SystemExit(
            f'ERROR: no existe {path}'
        )

    data = json.loads(
        path.read_text(encoding='utf-8')
    )

    value = data.get(
        'evaluation',
        {},
    ).get('initial_settle_steps')

    print(
        f'{path}: initial_settle_steps = {value}'
    )

    if value != 2:
        raise SystemExit(
            'ERROR: initial_settle_steps debe ser 2.'
        )

print('Configuración de evaluación verificada.')
PY
}


show_summary()
{
    echo
    echo "============================================================"
    echo "                    RESUMEN FINAL"
    echo "============================================================"

    printf "Individuos evaluados: "

    grep -cE \
        'Individuo [0-9]+: OF=' \
        "${DE_LOG}" \
        2>/dev/null \
        || true

    printf "Reciclados PID completados: "

    grep -c \
        'Reciclado PID del reposicionamiento' \
        "${LAUNCH_LOG}" \
        2>/dev/null \
        || true

    printf "Confirmaciones recibidas: "

    grep -c \
        'Confirmación PID #' \
        "${LAUNCH_LOG}" \
        2>/dev/null \
        || true

    echo
    echo "Últimos resultados registrados:"

    grep -E \
        'Individuo [0-9]+: OF=|Generación|Mejor|Best|best' \
        "${DE_LOG}" \
        2>/dev/null \
        | tail -n 20 \
        || true

    echo
    echo "Directorio de resultados:"
    echo "${RESULTS_DIRECTORY}"

    echo
    echo "Registro de Gazebo:"
    echo "${LAUNCH_LOG}"

    echo
    echo "Registro del algoritmo DE:"
    echo "${DE_LOG}"

    echo "============================================================"
}


cleanup()
{
    stop_simulation
}


trap cleanup EXIT

trap '
    echo
    echo "Optimización interrumpida por el usuario."
    exit 130
' INT TERM


echo "============================================================"
echo "       OPTIMIZACIÓN PID CON EVOLUCIÓN DIFERENCIAL"
echo "============================================================"
echo
echo "Identificador: ${RUN_ID}"
echo "Resultados:    ${RESULTS_DIRECTORY}"
echo

verify_configuration

echo
echo "Comprobando que no exista otra simulación..."

if ros2 service list 2>/dev/null |
    grep -qx '/world/empty/control'
then
    echo
    echo "ERROR: ya existe una simulación de Gazebo ejecutándose."
    echo "Ciérrala con Ctrl+C antes de iniciar este script."
    exit 1
fi

echo "No se detectó otra simulación."

echo
echo "Iniciando Gazebo pausado..."

setsid ros2 launch \
    "${PACKAGE}" \
    "${LAUNCH_FILE}" \
    > "${LAUNCH_LOG}" 2>&1 &

LAUNCH_PID=$!

echo "PID del launch: ${LAUNCH_PID}"

echo
echo "Esperando Gazebo y los controladores inactivos..."

if ! wait_for_controllers; then
    exit 1
fi

echo
echo "Controladores preparados:"
ros2 control list_controllers
echo
echo "La física permanece pausada."
echo "El nodo DE realizará la activación inicial."
echo
echo "Iniciando la optimización completa..."
echo

set +e

ros2 run \
    "${PACKAGE}" \
    "${DE_EXECUTABLE}" \
    "${RUN_ID}" \
    2>&1 | tee "${DE_LOG}"

DE_STATUS=${PIPESTATUS[0]}

set -e

echo
echo "Código de salida del algoritmo DE: ${DE_STATUS}"

show_summary

if [[ "${DE_STATUS}" -ne 0 ]]; then
    echo
    echo "ERROR: la optimización no terminó normalmente."
    exit "${DE_STATUS}"
fi

echo
echo "============================================================"
echo "OPTIMIZACIÓN TERMINADA CORRECTAMENTE"
echo "============================================================"

exit 0
