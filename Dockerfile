# syntax=docker/dockerfile:1
# Docker-образ команды для Hackathon Go1 (задания T1, T2).
#
# Требования ТЗ п. 3.1: linux/amd64, Ubuntu 22.04 LTS, ROS 2 Humble,
# Eclipse CycloneDDS (rmw_cyclonedds_cpp), только CPU, запуск без root,
# автономность без интернета во время испытаний.
#
# Базовый образ организаторов закрывает всё перечисленное: Ubuntu 22.04 +
# ROS 2 Humble + CycloneDDS + пользователь ros (UID/GID 1000). Он публично
# доступен из registry.cookcloud.ru без авторизации.

# Пин по digest тега v1.0.1 — сборка воспроизводима, тег у организаторов может перезаписаться.
FROM --platform=linux/amd64 registry.cookcloud.ru/base_images/ros2-base-humble-amd64@sha256:4d4a184901f154849d59477a077ce507f31fdf63b8bedd8caea460607eec243d

# RMW по требованию ТЗ (запускатель организаторов передаёт то же значение
# переменной в контейнер; совпадающие значения ничего не ломают).
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Явный bash для RUN: ниже используется `source` (в dash его нет).
SHELL ["/bin/bash", "-c"]

# ---- T2: MuJoCo, T4: RTAB-Map и robot_localization -------------------------
# Версия mujoco и модель Menagerie совпадают с судейской средой
# (ТЗ T2 п. 4.3: MuJoCo 3.3.7, Menagerie rev 0059d433); numpy < 2 требуется
# самим mujoco 3.3.7. RTAB-Map и robot_localization ставятся из репозиториев
# ROS. Сеть нужна только на этапе сборки.
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rtabmap-ros \
        ros-humble-robot-localization \
        ros-humble-image-transport \
        ros-humble-cv-bridge \
        python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir "numpy<2" "mujoco==3.3.7"

# ---- T3: инференс RL-политики -----------------------------------------------
# torch только для CPU: интернета во время испытаний нет, CUDA не используется.
# Пины packaging/setuptools — требования колёс torch CPU без конфликтов версий.
RUN python3 -m pip install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --no-cache-dir --force-reinstall "packaging<24" "setuptools<70"

# ---- Рабочее пространство участника ----------------------------------------
# Путь совпадает с конвенцией публичного симулятора (/opt/go1_participant/...),
# чтобы локальные проверки находили unitree_go без дополнительных настроек.
RUN mkdir -p /opt/go1_participant && chown -R ros:ros /opt/go1_participant

COPY --chown=ros:ros ros2_ws /opt/go1_participant/ros2_ws
COPY --chown=ros:ros models/unitree_go1 /opt/go1_participant/models/unitree_go1

USER ros
RUN source /opt/ros/humble/setup.bash \
    && cd /opt/go1_participant/ros2_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Проверки на этапе сборки: интерфейсы unitree_go собрались, модель и пары
# читаются, все 655 проверяемых пар на месте; веса RL-политики установлены
# в share пакета (оттуда их читает rl_node) и torch импортируется.
RUN source /opt/ros/humble/setup.bash \
    && source /opt/go1_participant/ros2_ws/install/setup.bash \
    && python3 -c "from unitree_go.msg import LowCmd, LowState; assert len(LowCmd().motor_cmd) == 20" \
    && python3 -c "from safety_block.collision import CollisionChecker; c = CollisionChecker('/opt/go1_participant/models/unitree_go1'); assert len(c.pairs) == 655, len(c.pairs)" \
    && python3 -c "import torch" \
    && test -f /opt/go1_participant/ros2_ws/install/rl_controller/share/rl_controller/wtw/body_latest.jit

# Контракт запуска: исполняемый /entrypoint.sh в корне файловой системы.
USER root
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh \
    && bash -n /entrypoint.sh

# По умолчанию — не root (пользователь ros из базового образа).
USER ros

# ENTRYPOINT объявляем для самодостаточности образа: судейский запускатель
# (participant_runtime.py) в любом случае перекрывает его тем же путём
# (--entrypoint /entrypoint.sh), а при локальном запуске без флага
# наследуемый entrypoint базового образа (/ros2_custom_entrypoint.sh)
# не должен перехватывать команды — он рассчитан на Simulator.
ENTRYPOINT ["/entrypoint.sh"]

# Аргумент по умолчанию не задаём: запускатель всегда передаёт его явно
# (ping либо идентификатор задания), CMD без аргумента дал бы usage-ошибку.
