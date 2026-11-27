#!/usr/bin/env bash
# Стенд T3: N эпизодов по протоколу судьи (ТЗ T3 п. 5.3) — каждый эпизод
# на полностью пересозданной паре контейнеров Simulator + Participant.
# Оркестратор стенда, НЕ часть решения (в образ не входит): Judge здесь
# имитируется t3_selfcheck.py, который кладётся в контейнер симулятора.
#
# Запуск: tools/t3_episodes.sh [ЭПИЗОДОВ] [СКОРОСТЬ]
#   ЭПИЗОДОВ — по умолчанию 10; СКОРОСТЬ — м/с, по умолчанию 0.3
#   (= preferred_speed_mps из entrypoint.sh; менять их синхронно).
# Требует: docker, симулятор в $SIM_DIR (по умолчанию ~/go1-simulator-public),
# образ участника $IMAGE (по умолчанию go1-participant:t3).
#
# Журнал: $OUT (по умолчанию /tmp/t3-episodes-<метка>.log), рядом файлы
# .runN (лог супервизора) и .epN (лог эпизода). В конце — сводка X/N.
set -u

SIM_DIR="${SIM_DIR:-$HOME/go1-simulator-public}"
IMAGE="${IMAGE:-go1-participant:t3}"
N_EPISODES="${1:-10}"
SPEED="${2:-0.3}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SELFCK="$HERE/t3_selfcheck.py"
OUT="${OUT:-/tmp/t3-episodes-$(date +%Y%m%d-%H%M%S).log}"

case "$N_EPISODES$SPEED" in *[!0-9.]*) echo "ЭПИЗОДОВ и СКОРОСТЬ — числа" >&2; exit 2;; esac
for need in "$SIM_DIR/run_local.py" "$SELFCK"; do
    [ -f "$need" ] || { echo "нет файла: $need" >&2; exit 2; }
done
docker image inspect "$IMAGE" >/dev/null 2>&1 || { echo "нет образа: $IMAGE" >&2; exit 2; }

log() { printf '%s\n' "$*" | tee -a "$OUT"; }
log "STAND START $(date '+%F %T') image=$IMAGE episodes=$N_EPISODES speed=$SPEED"
SUCCESS=0

for i in $(seq 1 "$N_EPISODES"); do
    log "--- EPISODE $i/$N_EPISODES $(date '+%F %T')"

    # Чистка хвостов: обрыв супервизора оставляет пару контейнеров висеть.
    stale=$(docker ps -aq --filter name=go1-local-)
    if [ -n "$stale" ]; then
        log "  cleanup: leftovers $stale"
        docker rm -f $stale >/dev/null
    fi

    # Супервизор в фоне, лог в файл (без пайпов на живой процесс).
    RUNLOG="${OUT}.run$i"
    python3 "$SIM_DIR/run_local.py" --task T3 --participant-image "$IMAGE" \
        --headless >"$RUNLOG" 2>&1 &
    SUP=$!

    # Готовность пары: маркер после readiness-пробы и старта участника.
    ready=""
    for _ in $(seq 1 180); do
        grep -q "^Pair started" "$RUNLOG" 2>/dev/null && { ready=1; break; }
        grep -q "^LOCAL error" "$RUNLOG" 2>/dev/null && break
        kill -0 "$SUP" 2>/dev/null || break
        sleep 1
    done
    if [ -z "$ready" ]; then
        log "  EPISODE $i: FAIL — супервизор не поднял пару"
        sed 's/^/    | /' "$RUNLOG" >> "$OUT"
        kill -TERM "$SUP" 2>/dev/null
        wait "$SUP" 2>/dev/null
        continue
    fi
    TOKEN=$(sed -n 's/^Run=\(go1-local-[0-9a-f]*\).*/\1/p' "$RUNLOG" | head -1)
    SIM="${TOKEN}-sim"; PART="${TOKEN}-participant"
    log "  пара готова: $TOKEN"

    # Готовность политики: маркер WTW Ready в логе rl_node (загрузка torch).
    rl_ok=""
    for _ in $(seq 1 120); do
        if ! docker inspect -f '{{.State.Running}}' "$PART" 2>/dev/null | grep -q true; then break; fi
        if docker exec "$PART" grep -q "WTW Ready" /tmp/rl_controller.log 2>/dev/null; then rl_ok=1; break; fi
        sleep 1
    done
    if [ -z "$rl_ok" ]; then
        log "  EPISODE $i: FAIL — rl_node не поднялся"
        docker logs --tail 30 "$PART" 2>&1 | sed 's/^/    | /' >> "$OUT"
        docker exec "$PART" tail -30 /tmp/rl_controller.log 2>/dev/null | sed 's/^/    rl| /' >> "$OUT"
        kill -TERM "$SUP" 2>/dev/null; wait "$SUP" 2>/dev/null
        continue
    fi
    log "  rl_node готов, эпизод пошёл"

    # Эпизод: selfcheck в контейнере симулятора, вердикт по коду выхода.
    docker cp "$SELFCK" "$SIM:/tmp/t3_selfcheck.py" >/dev/null
    if docker exec "$SIM" bash -c 'source /etc/go1_sim_env.sh 2>/dev/null || true; python3 -c "import rclpy" 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true; exec python3 /tmp/t3_selfcheck.py --speed '"$SPEED" >"${OUT}.ep$i" 2>&1; then
        SUCCESS=$((SUCCESS + 1)); verdict=SUCCESS
    else
        verdict=FAIL
    fi
    grep -E "EPISODE RESULT|forward=|/lowcmd|WARNING|error|Error" "${OUT}.ep$i" | sed 's/^/    /' >> "$OUT"
    log "  EPISODE $i: $verdict"

    # Остановка: SIGTERM у супервизора запускает штатный cleanup.
    kill -TERM "$SUP" 2>/dev/null
    for _ in $(seq 1 30); do kill -0 "$SUP" 2>/dev/null || break; sleep 1; done
    if kill -0 "$SUP" 2>/dev/null; then
        log "  cleanup: супервизор не завершился, жёсткая чистка"
        kill -KILL "$SUP" 2>/dev/null
        docker rm -f "$SIM" "$PART" >/dev/null 2>&1
    fi
    wait "$SUP" 2>/dev/null
    tail -1 "$RUNLOG" | sed 's/^/    /' >> "$OUT"
    sleep 3
done

log "=== STAND RESULT: $SUCCESS/$N_EPISODES SUCCESS $(date '+%F %T')"
log "журналы: $OUT ${OUT}.ep* ${OUT}.run*"
