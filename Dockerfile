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

FROM --platform=linux/amd64 registry.cookcloud.ru/base_images/ros2-base-humble-amd64:v1.0.1

# RMW по требованию ТЗ (запускатель организаторов передаёт то же значение
# переменной в контейнер; совпадающие значения ничего не ломают).
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Контракт запуска: исполняемый /entrypoint.sh в корне файловой системы.
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh \
    && bash -n /entrypoint.sh

# По умолчанию — не root (пользователь ros из базового образа).
USER ros

# Аргумент по умолчанию не задаём: запускатель всегда передаёт его явно
# (ping либо идентификатор задания), CMD без аргумента дал бы usage-ошибку.
