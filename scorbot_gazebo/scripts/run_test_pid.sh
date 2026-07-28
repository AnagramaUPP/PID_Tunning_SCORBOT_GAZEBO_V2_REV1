#!/usr/bin/env bash

set -o pipefail

WORKSPACE="${HOME}/ros2_ws"
PACKAGE="scorbot_gazebo"
LAUNCH_FILE="tuning.launch.py"
TEST_EXECUTABLE="test_fixed_pid.py"
TEST_NAME="${1:-valores_prueba}"

STAMP="$(date +%Y%m%d_%H%M%S)"
RESULTS_DIRECTORY="${WORKSPACE}/results/prueba_PID_${STAMP}"

mkdir -p "${RESULTS_DIRECTORY}"

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"

cd "${RESULTS_DIRECTORY}" || exit 1

LAUNCH_LOG="${RESULTS_DIRECTORY}/${TEST_NAME}_launch.log"
TEST_LOG="${RESULTS_DIRECTORY}/${TEST_NAME}_resultado.log"

LAUNCH_PID=""


stop_simulation()
{
    if [[ -z "${LAUNCH_PID}" ]]; then
        return
    fi

    echo
    echo "Cerrando Gazebo y los nodos de prueba..."

    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true

    for _ in $(seq 1 20); do
        if ! kill -0 -- "-${LAUNCH_PID}" 2>/dev/null; then
            break
        fi

        sleep 1
    done

    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true

    LAUNCH_PID=""
}


wait_for_controllers()
{
    for _ in $(seq 1 120); do

        if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            echo "El launch terminó antes de cargar los controladores."
            tail -n 60 "${LAUNCH_LOG}"
            return 1
        fi

        controllers="$(
            ros2 control list_controllers 2>/dev/null || true
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

        if [[ -n "${arm_ready}" && -n "${broadcaster_ready}" ]]; then
            return 0
        fi

        sleep 1
    done

    echo "Tiempo agotado esperando los controladores."
    tail -n 60 "${LAUNCH_LOG}"

    return 1
}


cleanup()
{
    stop_simulation
}


trap cleanup EXIT
trap 'echo; echo "Prueba interrumpida."; exit 130' INT TERM


echo "============================================================"
echo "          PRUEBA ÚNICA DE GANANCIAS PID"
echo "============================================================"
echo
echo "j1: 54.69044716   6.41544951   0.99772149"
echo "j2: 51.59466303   9.96681328   0.66609411"
echo "j3: 49.65080619   3.93376714   0.22416093"
echo "j4: 16.00559120   4.27910973   0.59188420"
echo "j5: 11.49211677   9.18247287   0.88279763"
echo
echo "Resultados: ${RESULTS_DIRECTORY}"
echo "============================================================"


# Gazebo inicia pausado y con controladores inactivos.
setsid ros2 launch \
    "${PACKAGE}" \
    "${LAUNCH_FILE}" \
    > "${LAUNCH_LOG}" 2>&1 &

LAUNCH_PID=$!

echo
echo "Esperando Gazebo y los controladores inactivos..."

if ! wait_for_controllers; then
    echo "ERROR: no fue posible preparar la simulación."
    exit 1
fi

echo "Controladores preparados."
echo "La física continúa pausada."
echo
echo "Cargando ganancias y realizando la evaluación única..."


ros2 run \
    "${PACKAGE}" \
    "${TEST_EXECUTABLE}" \
    "${TEST_NAME}" \
    2>&1 | tee "${TEST_LOG}"

TEST_STATUS=${PIPESTATUS[0]}


if [[ "${TEST_STATUS}" -ne 0 ]]; then
    echo
    echo "ERROR: la prueba terminó con código ${TEST_STATUS}."
    exit "${TEST_STATUS}"
fi


echo
echo "============================================================"
echo "PRUEBA PID TERMINADA CORRECTAMENTE"
echo "============================================================"
echo "Resultados almacenados en:"
echo "${RESULTS_DIRECTORY}"
echo "============================================================"
