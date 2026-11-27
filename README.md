# go1-solution — решение команды «Перспектива» (Hackathon Go1)

Команда: **Перспектива** (`perspektiva` — латиницей для Harbor).
GitHub: https://github.com/pr1mbl4/go1-solution

Решение заочного этапа хакатона Unitree Go1: Docker-образ **Participant**, который
инфраструктура организаторов запускает парой с контейнером Simulator/Judge.
Взаимодействие — через публичные интерфейсы ROS 2 (Humble, CycloneDDS).

Сейчас реализованы задания **T1 — подготовка сдачи решения (3 балла)**,
**T2 — Safety Block (12)**, **T3 — RL-ходьба (готовая политика, до 15)**,
**T4 — одометрия (20)** и **T5 — автономная навигация + ArUco (35)**.

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
3. **Rate-limit** — цель сдвигается не более чем на 4,99° от последней
   **опубликованной** цели (запас к 5° на округление). Это строже, чем
   «за принятую команду»: даже если между тиками публикации придёт несколько
   команд, соседние сообщения `/lowcmd` никогда не разойдутся больше чем на 5°.
4. **Самоколлизии** — переход от текущей цели к новой проверяется по
   `go1.xml` + `t2_collision_pairs.csv` (655 пар) с шагом 0,05° (тоньше
   судейских 0,1°), порог проникновения 1 мм. Опасный переход отклоняется,
   удерживается последняя безопасная цель.
5. **Watchdog** — если `/lowcmd_bare` молчит 0,45 с симуляционного времени,
   публикуется удержание: `q` = последняя принятая **цель**, `dq=0`, `tau=0`,
   `kp=40`, `kd=2`. Поток `/lowcmd` не прерывается.

Публикация: таймер 200 Гц по симуляционному времени (`use_sim_time`,
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

## T3 — RL-ходьба (готовая политика walk-these-ways)

Узел `rl_controller` читает `/lowstate` и `/cmd_vel`, публикует `/lowcmd_bare`
(дальше — Safety Block из T2, который формирует `/lowcmd`). Политика —
walk-these-ways, готовый чекпойнт для Go1: сеть body + adaptation module
(torchscript, инференс на CPU ~50 Гц по симуляционному времени), история
наблюдений 30 шагов. Политика не дообучалась; `self_trained_rl=false`.

Наблюдения собираются из `/lowstate`, `/cmd_vel` и собственной памяти прошлых
действий по схеме репозитория обучения: гравитация в системе корпуса, команды
с масштабами конфига, `q`/`dq` относительно дефолт-позы, два прошлых действия,
clock-входы походки. Кватернион `/lowstate` читается как `(w, x, y, z)`.

Детали, подобранные на стенде:

- kp/kd = 40/2 — совпадают с мягким стартом симулятора. Коэффициенты из
  конфига обучения (20/0.5) походку не держат, как и амплитуда действий
  `action_scale` из конфига (0.25) — политика обучена с actuator net, а не с
  PD; в узле стоит подобранная 0.18;
- при скорости команды ниже 0.05 м/с — режим стойки на дефолт-позе;
- курс удерживается PI-регулятором через штатный вход угловой скорости
  политики (команда судьи — движение без поворотов);
- веса ставятся в share пакета через `data_files` и читаются по
  `get_package_share_directory`.

Заявленная скорость `preferred_speed_mps` — 0.6 м/с (верх диапазона ТЗ):
политика систематически идёт медленнее команды, на 0.6 фактическая скорость
и запас по времени испытания максимальны.

Стенд (не входит в образ; Judge имитируется скриптом):

```bash
docker cp tools/t3_selfcheck.py <sim>:/tmp/
docker exec <sim> python3 /tmp/t3_selfcheck.py --speed 0.6   # один эпизод
tools/t3_episodes.sh 10 0.6                                  # серия: на каждый
# эпизод — полностью пересозданная пара Simulator + Participant
```

## T4 — Одометрия

RTAB-Map (`rgbd_odometry`) по фронтальной RGB-D → узел `odom_gate` (удержание
позы в покое по гироскопу и скоростям суставов; гасит дрейф визуальной
одометрии стоящего робота) → EKF `robot_localization`. Выход фильтра
ремапится в `/odom` + TF `odom→trunk`; конфиг фильтра —
`ros2_ws/src/odometry/config/ekf.yaml`.

Стенд: `tools/t4_selfcheck.py` — RPE по `/odom/ground_truth` и дрейф за окно
покоя.

## T5 — Автономная навигация + ArUco

Кейс T5 поднимает одометрию T4 (RTAB-Map → гейт покоя → EKF: `/odom` + TF
`odom→trunk`) и поверх неё — навигационный стек и детектор меток:

- **SLAM** — slam_toolbox (online async): карта строится в движении, узел
  публикует TF `map→odom`; цепочка `map→odom→trunk` полная, карта не
  сохраняется (по ТЗ не требуется);
- **Nav2** — bt_navigator с action-сервером `/navigate_to_pose` (цель от
  Judge), planner_server, smoother и контроллер Regulated Pure Pursuit;
  costmap и footprint настроены под коридоры 1 м. Узел `goal_relay`
  дополнительно принимает `PoseStamped` на `/goal_pose` (RViz) и после
  активации Nav2 переправляет цель в action;
- **ArUco** — `r8_aruco_detector`: словарь DICT_4X4_50, изображения всех
  пяти камер T5 (`/camera`, `/camera_chin`, `/camera_left`, `/camera_right`,
  `/camera_rearDown`; QoS RELIABLE, depth 5 — как в ТЗ). Обнаружение
  подтверждается тремя последовательными кадрами одной камеры и живёт 0,75 с
  после последнего кадра; результат — `/aruco/detections`
  (std_msgs/String, JSON `{"ids": [...]}`) только с реально видимыми в
  моменте метками, без перечисления всего словаря.

Раскладка: `ros2_ws/src/go1_nav/` (launch, `config/nav2_params.yaml`,
behavior XML, `goal_relay`), `ros2_ws/src/r8_aruco_detector/`.

Стенд (не входит в образ; цель шлётся action-клиентом, как Judge):

```bash
docker cp tools/t5_send_goal.py <participant>:/tmp/
docker exec <participant> bash -c \
  'source /opt/ros/humble/setup.bash && \
   source /opt/go1_participant/ros2_ws/install/setup.bash && \
   python3 /tmp/t5_send_goal.py --x 4.0 --y 0.0 --yaw 0'
```

Smoke-прогон на публичном курсе 1,0 м: цель достигнута с ошибкой 0,354 м и
−15° по курсу (лимиты ТЗ — 0,5 м / 30°), за прогон распознаны метки 0–3,
спама идентификаторов нет.

## Состав проекта

- `Dockerfile` — образ на базе организаторского
  `registry.cookcloud.ru/base_images/ros2-base-humble-amd64:v1.0.1`
  (в Dockerfile запинен по digest `sha256:4d4a1849…eec243d`;
  Ubuntu 22.04 + ROS 2 Humble + CycloneDDS + пользователь `ros`, UID 1000);
- `entrypoint.sh` — контракт запуска: аргументы `ping`, `T2`, `T3`, `T4`, `T5`;
- `ros2_ws/` — рабочее пространство участника: `unitree_go` (сообщения),
  `safety_block` (узел T2), `rl_controller` (RL-ходьба T3),
  `odometry` (гейт покоя T4), `go1_nav` (SLAM + Nav2 + goal_relay, T5),
  `r8_aruco_detector` (метки T5);
- `models/unitree_go1/` — модель Go1 и перечень пар для проверки коллизий;
- `tools/` — стенды самопроверки T2–T5 (в образ не входят);
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
- `T2` — Safety Block (`safety_node`): `/lowcmd_bare` → проверки → `/lowcmd`
  не реже 50 Гц;
- `T3` — RL-политика (`rl_node`) + Safety Block, обоим узлам задан
  `use_sim_time`;
- `T4` — RTAB-Map + гейт покоя + EKF (выход фильтра ремапится в `/odom`);
- `T5` — одометрия T4 и ArUco-детектор в фоне, затем
  `ros2 launch go1_nav t5.launch.py` (slam_toolbox + Nav2);
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
