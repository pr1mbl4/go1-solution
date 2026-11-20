# go1-solution — решение команды «Перспектива» (Hackathon Go1)

Команда: **Перспектива** (`perspektiva` — латиницей для Harbor).
GitHub: https://github.com/pr1mbl4/go1-solution

Решение заочного этапа хакатона Unitree Go1: Docker-образ **Participant**, который
инфраструктура организаторов запускает парой с контейнером Simulator/Judge.
Взаимодействие — через публичные интерфейсы ROS 2 (Humble, CycloneDDS).

Сейчас реализованы задания **T1 — подготовка сдачи решения (3 балла)** и
**T2 — Safety Block (12 баллов)**.

## Требования T1 (по ТЗ)

| Критерий | Проверка |
|---|---|
| Образ доступен по SHA-256 digest, архитектура `linux/amd64` | 1 балл |
| Корректный `/entrypoint.sh ping`: код 0, ≤ 10 с, ровно один JSON в stdout | 1 балл |
| Работа без интернета и загрузки внешних зависимостей | 1 балл |

Требования к образу: Ubuntu 22.04 LTS, ROS 2 Humble, `rmw_cyclonedds_cpp`,
только CPU, запуск без прав root, всё содержимое внутри образа.

## T2 — Safety Block

Узел `safety_block` читает `/lowcmd_bare`, проверяет команды и публикует
безопасный поток `/lowcmd` (судья оценивает только его).

Конвейер проверок (в этом порядке):

1. **NaN/±Inf** в любом из полей `q, dq, tau, kp, kd` первых 12 суставов —
   сообщение отбрасывается целиком, продолжается публикация последней
   принятой безопасной команды.
2. **Лимиты суставов** — `q` клипается в диапазоны ТЗ (hip ±0,863;
   thigh −0,686…4,501; calf −2,818…−0,888).
3. **Rate-limit** — цель сдвигается не более чем на 4,99° за принятую команду
   (запас к 5° на округление).
4. **Самоколлизии** — переход от текущей цели к новой проверяется по
   `go1.xml` + `t2_collision_pairs.csv` (655 пар) с шагом 0,05° (тоньше
   судейских 0,1°), порог проникновения 1 мм. Опасный переход отклоняется,
   удерживается последняя безопасная цель.
5. **Watchdog** — если `/lowcmd_bare` молчит 0,45 с симуляционного времени,
   публикуется удержание: `q` = последняя принятая **цель**, `dq=0`, `tau=0`,
   `kp=40`, `kd=2`. Поток `/lowcmd` не прерывается.

Публикация: таймер 100 Гц по симуляционному времени (`use_sim_time`,
`/clock`), `mode=0x0A` для суставов 0–11, QoS RELIABLE depth 1.
Подписка на `/lowcmd_bare` — BEST_EFFORT depth 10 (принимает издателей с
любой надёжностью).

Раскладка:

- `ros2_ws/src/safety_block/safety_block/logic.py` — вся логика без ROS
  (тестируется обычным pytest);
- `ros2_ws/src/safety_block/safety_block/collision.py` — MuJoCo-проверка
  конфигураций и переходов;
- `ros2_ws/src/safety_block/safety_block/safety_node.py` — обёртка rclpy;
- `ros2_ws/src/unitree_go/` — интерфейсы `unitree_go` (вендорены из
  `unitree_ros2` @ `668d1ec5`, только пакет сообщений);
- `models/unitree_go1/` — эталонная модель Menagerie rev `0059d433` и
  `t2_collision_pairs.csv` из публичной симуляции.

Локальные тесты логики (без ROS, нужен только `mujoco`):

```bash
cd ros2_ws/src/safety_block && python3 -m pytest test/test_logic.py
```

Сквозная проверка на стенде (внутри запущенной пары Simulator + Participant):

```bash
docker cp tools/t2_selfcheck.py <participant>:/tmp/t2_selfcheck.py
docker exec -it <participant> bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/go1_participant/ros2_ws/install/setup.bash \
   && python3 /tmp/t2_selfcheck.py'
```

Стенд подаёт безопасные/рывковые/NaN-команды, обрывает поток, шлёт опасную
конфигурацию и проверяет частоту, интервалы, лимиты, rate-limit, удержание,
отсутствие NaN и коллизий на выходе. Код 0 — все проверки пройдены.

## Состав проекта

- `Dockerfile` — образ на базе организаторского
  `registry.cookcloud.ru/base_images/ros2-base-humble-amd64:v1.0.1`
  (в Dockerfile запинен по digest `sha256:4d4a1849…eec243d`;
  Ubuntu 22.04 + ROS 2 Humble + CycloneDDS + пользователь `ros`, UID 1000);
- `entrypoint.sh` — контракт запуска: аргументы `ping`, `T2`, `T3`, `T4`, `T5`;
- `ros2_ws/` — рабочее пространство участника: `unitree_go` (сообщения) и
  `safety_block` (узел T2);
- `models/unitree_go1/` — модель Go1 и перечень пар для проверки коллизий;
- `check_t1.sh` — локальная проверка образа по всем критериям ТЗ T1;
- `README.md` — этот файл.

## Как собрать

Нужен Docker Engine с Buildx (архитектура хоста x86_64).

```bash
docker pull registry.cookcloud.ru/base_images/ros2-base-humble-amd64@sha256:4d4a184901f154849d59477a077ce507f31fdf63b8bedd8caea460607eec243d
docker build --platform linux/amd64 -t go1-participant:t1 .
```

## Как проверить

```bash
./check_t1.sh go1-participant:t1
```

Скрипт проверяет: платформу образа, ping (код выхода, время, строго один JSON),
работу ping без сети (`--network none`), не-root пользователя, исполнимость
`/entrypoint.sh`, реакцию на неизвестный аргумент.

## Логика entrypoint.sh

- `ping` — печатает JSON протокола (`protocol_version` 1.0, `supported_tasks`,
  `self_trained_rl`, `preferred_speed_mps`) и завершается с кодом 0.
  Значения ответа — три переменные в начале файла; ROS-окружение не
  задействуется, чтобы stdout оставался чистым.
- `T2`/`T3`/`T4`/`T5` — не реализованы: диагностика в stderr, код 69.
- Прочее — usage в stderr, код 64.

## Сдача решения

1. `git push` — исходники в публичный GitHub-репозиторий, зафиксировать коммит:
   `git rev-parse HEAD`.
2. `docker login registry.cookcloud.ru -u perspektiva` (учётка команды от организаторов;
   проект команды в Harbor — «perspektiva»).
3. Публикация:
   ```bash
   IMAGE="registry.cookcloud.ru/perspektiva/perspektiva:latest"
   docker buildx build --platform linux/amd64 -t "$IMAGE" --push .
   ```
4. Получить digest: `docker buildx imagetools inspect "$IMAGE"` → `Digest: sha256:…`
5. Проверить загрузку: `docker pull --platform linux/amd64 <repo>@<digest>`
6. В личный кабинет: ссылка на коммит GitHub + команда `docker pull` по digest.

Перед пересдачей при необходимости обновить `supported_tasks` в `entrypoint.sh`,
пересобрать и опубликовать — в кабинет вносится **новый digest**.
