# syntax=docker/dockerfile:1
# Docker-образ команды для Hackathon Go1 (задание T1 — подготовка сдачи решения).
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

# Контракт запуска: исполняемый /entrypoint.sh в корне файловой системы.
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
