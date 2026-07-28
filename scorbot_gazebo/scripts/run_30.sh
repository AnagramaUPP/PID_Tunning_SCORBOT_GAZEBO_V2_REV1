#!/usr/bin/env bash

set -o pipefail

WORKSPACE="${HOME}/ros2_ws"
PACKAGE="scorbot_gazebo"
SINGLE_RUN_SCRIPT="run_tuning_de.sh"

TOTAL_RUNS="${TOTAL_RUNS:-30}"
BASE_NAME="${1:-corrida_DE}"
PAUSE_BETWEEN_RUNS="${PAUSE_BETWEEN_RUNS:-5}"

if ! [[ "${TOTAL_RUNS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: TOTAL_RUNS debe ser un entero positivo."
    exit 1
fi

if ! [[ "${PAUSE_BETWEEN_RUNS}" =~ ^[0-9]+$ ]]; then
    echo "ERROR: PAUSE_BETWEEN_RUNS debe ser un entero."
    exit 1
fi

if [[ ! "${BASE_NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "ERROR: el nombre solo puede contener:"
    echo "letras, números, guion y guion bajo."
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"

BATCH_STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_ID="${BASE_NAME}_${BATCH_STAMP}"

BATCH_DIRECTORY="${WORKSPACE}/results/30_corridas_${BATCH_ID}"
SUMMARY_FILE="${BATCH_DIRECTORY}/resumen_30_corridas.csv"
MASTER_LOG="${BATCH_DIRECTORY}/${BATCH_ID}_master.log"
STATE_FILE="${BATCH_DIRECTORY}/estado.txt"

mkdir -p "${BATCH_DIRECTORY}"

CURRENT_RUN=0
CURRENT_PROCESS_PID=""
INTERRUPTED=0


timestamp()
{
    date '+%Y-%m-%d %H:%M:%S'
}


log()
{
    printf '[%s] %s\n' "$(timestamp)" "$*" |
        tee -a "${MASTER_LOG}"
}


close_previous_simulation()
{
    if ros2 service list 2>/dev/null |
        grep -qx '/world/empty/control'
    then
        log "ERROR: ya existe una simulación de Gazebo."
        log "Ciérrala antes de ejecutar run_30.sh."
        return 1
    fi

    return 0
}


find_latest_run_directory()
{
    local run_name="$1"

    find "${WORKSPACE}/results" \
        -mindepth 1 \
        -maxdepth 1 \
        -type d \
        -name "tuning_DE_${run_name}_*" \
        -printf '%T@ %p\n' \
        2>/dev/null |
        sort -nr |
        head -n 1 |
        cut -d' ' -f2-
}


extract_best_result()
{
    local run_directory="$1"
    local de_log
    local best_file
    local best_of=""
    local best_scv=""
    local individuals="0"

    de_log="$(
        find "${run_directory}" \
            -maxdepth 1 \
            -type f \
            -name '*_de.log' \
            | head -n 1
    )"

    best_file="$(
        find "${run_directory}" \
            -maxdepth 1 \
            -type f \
            -name 'best_pid_values_*.txt' \
            | head -n 1
    )"

    if [[ -n "${de_log}" && -f "${de_log}" ]]; then
        individuals="$(
            grep -cE \
                'Individuo [0-9]+: OF=' \
                "${de_log}" \
                2>/dev/null \
                || true
        )"

        read -r best_of best_scv < <(
            python3 - "${de_log}" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(
    encoding='utf-8',
    errors='replace',
)

matches = re.findall(
    r'X_best(?: founded)?:\s*\[(.*?)\]',
    text,
    flags=re.DOTALL,
)

if not matches:
    print("", "")
    raise SystemExit

numbers = [
    float(value)
    for value in re.findall(
        r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)'
        r'(?:[eE][-+]?\d+)?',
        matches[-1],
    )
]

if len(numbers) >= 17:
    print(numbers[-2], numbers[-1])
else:
    print("", "")
PY
        )
    fi

    printf '%s|%s|%s|%s\n' \
        "${individuals}" \
        "${best_of}" \
        "${best_scv}" \
        "${best_file}"
}


write_state()
{
    cat > "${STATE_FILE}" <<EOF
batch_id=${BATCH_ID}
total_runs=${TOTAL_RUNS}
current_run=${CURRENT_RUN}
interrupted=${INTERRUPTED}
updated_at=$(timestamp)
EOF
}


cleanup()
{
    local status=$?

    INTERRUPTED=1
    write_state

    if [[ -n "${CURRENT_PROCESS_PID}" ]] &&
       kill -0 "${CURRENT_PROCESS_PID}" 2>/dev/null
    then
        log "Interrumpiendo la corrida ${CURRENT_RUN}..."

        kill -INT "${CURRENT_PROCESS_PID}" 2>/dev/null || true

        for _ in $(seq 1 20); do
            if ! kill -0 "${CURRENT_PROCESS_PID}" 2>/dev/null; then
                break
            fi

            sleep 1
        done

        if kill -0 "${CURRENT_PROCESS_PID}" 2>/dev/null; then
            kill -TERM "${CURRENT_PROCESS_PID}" 2>/dev/null || true
        fi

        wait "${CURRENT_PROCESS_PID}" 2>/dev/null || true
    fi

    log "Proceso de 30 corridas detenido."
    log "Resumen parcial: ${SUMMARY_FILE}"

    exit "${status}"
}


trap cleanup INT TERM


if ! close_previous_simulation; then
    exit 1
fi


echo \
'corrida,identificador,inicio,fin,duracion_segundos,estado,codigo_salida,individuos,best_of,best_scv,directorio_resultados,archivo_best' \
> "${SUMMARY_FILE}"


log "============================================================"
log "       EJECUCIÓN DE ${TOTAL_RUNS} CORRIDAS DE"
log "          EVOLUCIÓN DIFERENCIAL PARA PID"
log "============================================================"
log "Lote: ${BATCH_ID}"
log "Directorio maestro: ${BATCH_DIRECTORY}"
log "Pausa entre corridas: ${PAUSE_BETWEEN_RUNS} s"
log "============================================================"

write_state


for RUN_NUMBER in $(seq 1 "${TOTAL_RUNS}"); do

    CURRENT_RUN="${RUN_NUMBER}"
    write_state

    RUN_INDEX="$(
        printf '%02d' "${RUN_NUMBER}"
    )"

    RUN_NAME="${BASE_NAME}_${RUN_INDEX}"

    RUN_START_EPOCH="$(date +%s)"
    RUN_START_TEXT="$(timestamp)"

    RUN_WRAPPER_LOG="${BATCH_DIRECTORY}/${RUN_NAME}_wrapper.log"

    log
    log "------------------------------------------------------------"
    log "CORRIDA ${RUN_NUMBER}/${TOTAL_RUNS}"
    log "Identificador base: ${RUN_NAME}"
    log "Inicio: ${RUN_START_TEXT}"
    log "------------------------------------------------------------"

    set +e

    ros2 run \
        "${PACKAGE}" \
        "${SINGLE_RUN_SCRIPT}" \
        "${RUN_NAME}" \
        > >(
            tee "${RUN_WRAPPER_LOG}" |
            tee -a "${MASTER_LOG}"
        ) \
        2>&1 &

    CURRENT_PROCESS_PID=$!

    wait "${CURRENT_PROCESS_PID}"
    RUN_STATUS=$?

    CURRENT_PROCESS_PID=""

    set -e

    RUN_END_EPOCH="$(date +%s)"
    RUN_END_TEXT="$(timestamp)"
    RUN_DURATION="$((RUN_END_EPOCH - RUN_START_EPOCH))"

    RUN_DIRECTORY="$(
        find_latest_run_directory "${RUN_NAME}"
    )"

    INDIVIDUALS="0"
    BEST_OF=""
    BEST_SCV=""
    BEST_FILE=""

    if [[ -n "${RUN_DIRECTORY}" ]]; then
        RESULT_DATA="$(
            extract_best_result "${RUN_DIRECTORY}"
        )"

        IFS='|' read -r \
            INDIVIDUALS \
            BEST_OF \
            BEST_SCV \
            BEST_FILE \
            <<< "${RESULT_DATA}"
    fi

    if [[ "${RUN_STATUS}" -eq 0 ]]; then
        RUN_STATE="OK"

        log "Corrida ${RUN_NUMBER} terminada correctamente."
        log "Duración: ${RUN_DURATION} s"
        log "Individuos: ${INDIVIDUALS}"
        log "Mejor OF: ${BEST_OF:-no disponible}"
        log "SCV: ${BEST_SCV:-no disponible}"
    else
        RUN_STATE="ERROR"

        log "ERROR: corrida ${RUN_NUMBER} terminó"
        log "con código ${RUN_STATUS}."
    fi

    printf \
        '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "${RUN_NUMBER}" \
        "${RUN_NAME}" \
        "${RUN_START_TEXT}" \
        "${RUN_END_TEXT}" \
        "${RUN_DURATION}" \
        "${RUN_STATE}" \
        "${RUN_STATUS}" \
        "${INDIVIDUALS}" \
        "${BEST_OF}" \
        "${BEST_SCV}" \
        "${RUN_DIRECTORY}" \
        "${BEST_FILE}" \
        >> "${SUMMARY_FILE}"

    if [[ "${RUN_STATUS}" -ne 0 ]]; then
        log
        log "Se detienen las corridas para no continuar"
        log "después de una ejecución fallida."
        log "Corrige el problema y revisa:"
        log "${RUN_WRAPPER_LOG}"

        INTERRUPTED=1
        write_state
        exit "${RUN_STATUS}"
    fi

    write_state

    if [[ "${RUN_NUMBER}" -lt "${TOTAL_RUNS}" ]]; then
        log "Esperando ${PAUSE_BETWEEN_RUNS} s antes"
        log "de iniciar la siguiente corrida..."

        sleep "${PAUSE_BETWEEN_RUNS}"

        if ros2 service list 2>/dev/null |
            grep -qx '/world/empty/control'
        then
            log "ERROR: Gazebo continúa activo después"
            log "de terminar la corrida ${RUN_NUMBER}."

            INTERRUPTED=1
            write_state
            exit 1
        fi
    fi
done


INTERRUPTED=0
write_state


log
log "============================================================"
log "          LAS ${TOTAL_RUNS} CORRIDAS TERMINARON"
log "============================================================"
log "Resumen CSV:"
log "${SUMMARY_FILE}"
log
log "Registro maestro:"
log "${MASTER_LOG}"
log "============================================================"


python3 - "${SUMMARY_FILE}" <<'PY'
from pathlib import Path
import csv
import math
import statistics
import sys

path = Path(sys.argv[1])

with path.open(
    encoding='utf-8',
    newline='',
) as file:
    rows = list(csv.DictReader(file))

successful = [
    row
    for row in rows
    if row['estado'] == 'OK'
]

objective_values = []

for row in successful:
    try:
        value = float(row['best_of'])
    except (TypeError, ValueError):
        continue

    if math.isfinite(value):
        objective_values.append(value)

print()
print('========== RESUMEN ESTADÍSTICO ==========')
print(f'Corridas registradas: {len(rows)}')
print(f'Corridas correctas:   {len(successful)}')
print(
    f'Corridas con error:   '
    f'{len(rows) - len(successful)}'
)

if objective_values:
    print(
        f'Mejor OF global:      '
        f'{min(objective_values):.6f}'
    )
    print(
        f'OF promedio:          '
        f'{statistics.mean(objective_values):.6f}'
    )

    if len(objective_values) > 1:
        print(
            f'Desviación estándar:  '
            f'{statistics.stdev(objective_values):.6f}'
        )

    print(
        f'Peor OF:              '
        f'{max(objective_values):.6f}'
    )
else:
    print(
        'No se pudieron extraer valores OF '
        'de los registros.'
    )

print('=========================================')
PY

exit 0
