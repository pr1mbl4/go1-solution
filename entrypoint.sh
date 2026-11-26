#!/bin/bash
# Entrypoint контейнера Participant — Hackathon Go1, задание T1.
# Контракт запуска (ТЗ п. 3.2): образ запускается инфраструктурой организаторов
# с аргументом ping | T2 | T3 | T4 | T5. Путь фиксирован: /entrypoint.sh.
#
# Протокол ping (ТЗ п. 3.3):
#   - завершение с кодом 0 в течение 10 секунд реального времени;
#   - в stdout РОВНО один корректный JSON-объект (допустимы только пробельные
#     символы вокруг), любой другой текст в stdout запрещён;
#   - диагностика — только в stderr.
#
# Файл запускается БЕЗ прав root (пользователь ros, UID 1000).

# ---- Параметры ответа ping (править здесь перед пересдачей) ---------------
SUPPORTED_TASKS='["T2", "T3", "T4"]'  # перечень реализованных заданий, напр. ["T2"]
SELF_TRAINED_RL="false"         # true — RL-политика обучена командой
PREFERRED_SPEED_MPS="0.3"       # рекомендуемая скорость T3 в м/с или null
# ---------------------------------------------------------------------------

usage_to_stderr() {
    echo "usage: $0 ping | T2 | T3 | T4 | T5" >&2
}

case "${1:-}" in
    ping)
        # JSON собирается по шаблону: подставляются только переменные выше,
        # поэтому структура ответа фиксирована и всегда валидна.
        printf '{\n  "protocol_version": "1.0",\n  "supported_tasks": %s,\n  "self_trained_rl": %s,\n  "preferred_speed_mps": %s\n}\n' \
            "$SUPPORTED_TASKS" "$SELF_TRAINED_RL" "$PREFERRED_SPEED_MPS"
        exit 0
        ;;
    T2)
        # Safety Block: /lowcmd_bare -> проверки -> /lowcmd не реже 50 Гц.
        # use_sim_time обязателен: частота и watchdog считаются по /clock.
        source /opt/ros/humble/setup.bash
        source /opt/go1_participant/ros2_ws/install/setup.bash
        exec ros2 run safety_block safety_node --ros-args \
            -p use_sim_time:=true \
            -p model_dir:=/opt/go1_participant/models/unitree_go1
        ;;
    T3)
        # Ходьба: RL-политика -> /lowcmd_bare -> Safety Block -> /lowcmd.
        # use_sim_time у обоих узлов: частота инференса и watchdog по /clock.
        source /opt/ros/humble/setup.bash
        source /opt/go1_participant/ros2_ws/install/setup.bash

        # Safety Block: проверки T2, публикация /lowcmd >= 50 Гц
        ros2 run safety_block safety_node --ros-args \
            -p use_sim_time:=true \
            -p model_dir:=/opt/go1_participant/models/unitree_go1 \
            &> /tmp/safety_block.log &

        # RL-политика: /lowstate + /cmd_vel -> /lowcmd_bare
        ros2 run rl_controller rl_node --ros-args \
            -p use_sim_time:=true \
            &> /tmp/rl_controller.log &

        wait
        ;;
    T4)
        # Одометрия: RTAB-Map по фронтальной RGB-D -> гейт покоя -> EKF.
        # Выход фильтра ремапится в /odom, судья читает только его (ТЗ 6.2).
        source /opt/ros/humble/setup.bash
        source /opt/go1_participant/ros2_ws/install/setup.bash

        # RTAB-Map визуальная одометрия
        ros2 run rtabmap_odom rgbd_odometry --ros-args \
            --remap odom:=/visual_odom \
            --remap rgb/image:=/camera/image_raw \
            --remap depth/image:=/camera/depth/image_raw \
            --remap rgb/camera_info:=/camera/camera_info \
            -p use_sim_time:=true \
            -p frame_id:=trunk \
            -p odom_frame_id:=odom \
            -p publish_tf:=false \
            -p subscribe_depth:=true \
            -p subscribe_rgb:=true \
            -p approx_sync:=true \
            -p qos:=2 \
            -p Odom/Strategy:=\"0\" \
            -p Odom/FeatureType:=\"2\" \
            -p Odom/MaxFeatures:=\"500\" \
            &> /tmp/rtabmap.log &

        # Гейт покоя: удерживает позу, пока робот стоит
        ros2 run odometry odom_gate --ros-args \
            -p use_sim_time:=true \
            &> /tmp/odom_gate.log &

        # EKF: вход - /visual_odom_gated, выход - /odom + TF odom -> trunk
        exec ros2 run robot_localization ekf_node --ros-args \
            -p use_sim_time:=true \
            --params-file /opt/go1_participant/ros2_ws/install/odometry/share/odometry/config/ekf.yaml \
            --remap odometry/filtered:=/odom
        ;;
    T5)
        # Задание не реализовано: решение с пустым supported_tasks допущено
        # только к T1. Диагностика — в stderr, ненулевой код выхода.
        echo "task $1 is not implemented by this solution (supported_tasks=$SUPPORTED_TASKS)" >&2
        exit 69  # EX_UNAVAILABLE
        ;;
    *)
        usage_to_stderr
        exit 64  # EX_USAGE
        ;;
esac
