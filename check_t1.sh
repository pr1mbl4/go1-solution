#!/usr/bin/env bash
# Локальная проверка Docker-образа на соответствие критериям T1 (ТЗ п. 3.4).
# Использование: ./check_t1.sh [IMAGE]   (по умолчанию go1-participant:t1)
#
# Проверки:
#   1. Архитектура linux/amd64                     (критерий 1 ТЗ)
#   2. /entrypoint.sh ping: код 0, ровно один JSON в stdout, < 10 с (критерий 2)
#   3. ping без сети (--network none)              (критерий 3 ТЗ)
#   4. Запуск не от root
#   5. /entrypoint.sh существует и исполним
#   6. Неизвестный аргумент: код != 0, stdout пуст
set -u

IMAGE="${1:-go1-participant:t1}"
PASS=0
FAIL=0

ok()   { echo "  [ok]   $1"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
hdr()  { echo "== $1"; }

# --- 1. Архитектура ---------------------------------------------------------
hdr "1. Архитектура образа"
plat=$(docker image inspect "$IMAGE" --format '{{.Os}}/{{.Architecture}}' 2>/dev/null)
if [ "$plat" = "linux/amd64" ]; then
    ok "платформа $plat"
else
    bad "платформа '${plat:-образ не найден}', ожидалось linux/amd64"
fi

# --- 2. ping: код, время, ровно один JSON -----------------------------------
# Запуск имитирует судейский запускатель (participant_runtime.py): он всегда
# создаёт контейнер с --entrypoint /entrypoint.sh и аргументом задания.
hdr "2. /entrypoint.sh ping"
start=$(date +%s.%N)
ping_out=$(docker run --rm --entrypoint /entrypoint.sh "$IMAGE" ping 2>/dev/null)
code=$?
end=$(date +%s.%N)
elapsed=$(echo "$end $start" | awk '{printf "%.1f", $1-$2}')

if [ "$code" -eq 0 ]; then
    ok "код выхода 0"
else
    bad "код выхода $code"
fi

awk -v t="$elapsed" 'BEGIN { exit !(t < 10.0) }' \
    && ok "время выполнения ${elapsed}s (< 10s)" \
    || bad "время выполнения ${elapsed}s (>= 10s)"

# Ровно один корректный JSON: json.loads парсит ВЕСЬ текст и падает на лишнем.
# Сам ответ передаётся через переменную окружения, потому что stdin занят
# кодом проверки (python3 - читает программу из stdin-heredoc).
json_err=$(PING_JSON="$ping_out" python3 -I - 2>&1 <<'PYEOF'
import json, os, sys
raw = os.environ.get("PING_JSON", "")
try:
    d = json.loads(raw)
except Exception as e:
    print(f"не парсится как один JSON: {e}")
    sys.exit(1)
errs = []
if d.get("protocol_version") != "1.0":
    errs.append(f'protocol_version={d.get("protocol_version")!r}, нужно "1.0"')
st = d.get("supported_tasks")
if not (isinstance(st, list) and all(isinstance(x, str) and x in ("T2", "T3", "T4", "T5") for x in st)):
    errs.append(f"supported_tasks={st!r}, нужен array[string] из T2/T3/T4/T5")
if not isinstance(d.get("self_trained_rl"), bool):
    errs.append("self_trained_rl не boolean")
ps = d.get("preferred_speed_mps")
if ps is not None and (isinstance(ps, bool) or not isinstance(ps, (int, float))):
    errs.append(f"preferred_speed_mps={ps!r}, нужен number или null")
extra = set(d) - {"protocol_version", "supported_tasks", "self_trained_rl", "preferred_speed_mps"}
if extra:
    errs.append(f"лишние поля: {sorted(extra)}")
if errs:
    print("; ".join(errs))
    sys.exit(1)
PYEOF
)
if [ $? -eq 0 ]; then
    ok "stdout — один корректный JSON с полями протокола"
else
    bad "JSON: $json_err"
    echo "    stdout: $(printf '%s' "$ping_out" | head -c 500)"
fi

# --- 3. ping без сети --------------------------------------------------------
hdr "3. Работа без сети"
off_out=$(docker run --rm --entrypoint /entrypoint.sh --network none "$IMAGE" ping 2>/dev/null)
off_code=$?
if [ "$off_code" -eq 0 ] && [ "$off_out" = "$ping_out" ]; then
    ok "ping без сети: код 0, ответ идентичен"
else
    bad "ping без сети: код $off_code, вывод отличается"
fi

# --- 4. Не root ---------------------------------------------------------------
hdr "4. Права пользователя"
uid=$(docker run --rm --entrypoint id "$IMAGE" -u 2>/dev/null)
if [ "$uid" != "0" ]; then
    ok "контейнер работает от uid=$uid (не root)"
else
    bad "контейнер работает от root"
fi

# --- 5. /entrypoint.sh исполним ----------------------------------------------
hdr "5. Контракт запуска"
if docker run --rm --entrypoint sh "$IMAGE" -c 'test -x /entrypoint.sh' 2>/dev/null; then
    ok "/entrypoint.sh существует и исполним"
else
    bad "/entrypoint.sh отсутствует или не исполним"
fi

# --- 6. Неизвестный аргумент --------------------------------------------------
hdr "6. Неизвестный аргумент"
junk_out=$(docker run --rm --entrypoint /entrypoint.sh "$IMAGE" Bogus 2>/dev/null)
junk_code=$?
if [ "$junk_code" -ne 0 ] && [ -z "$junk_out" ]; then
    ok "код $junk_code (≠0), stdout пуст"
else
    bad "код $junk_code, stdout: '$junk_out'"
fi

echo "==="
echo "Пройдено: $PASS, провалено: $FAIL"
[ "$FAIL" -eq 0 ]
