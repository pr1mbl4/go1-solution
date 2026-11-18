# go1-solution — решение команды «Перспектива» (Hackathon Go1)

Команда: **Перспектива** (`perspektiva` — латиницей для Harbor).
GitHub: https://github.com/pr1mbl4/go1-solution

Решение заочного этапа хакатона Unitree Go1: Docker-образ **Participant**, который
инфраструктура организаторов запускает парой с контейнером Simulator/Judge.
Взаимодействие — через публичные интерфейсы ROS 2 (Humble, CycloneDDS).

Сейчас реализовано задание **T1 — подготовка сдачи решения (3 балла)**,
обязательное для допуска к T2–T5.

## Требования T1 (по ТЗ)

| Критерий | Проверка |
|---|---|
| Образ доступен по SHA-256 digest, архитектура `linux/amd64` | 1 балл |
| Корректный `/entrypoint.sh ping`: код 0, ≤ 10 с, ровно один JSON в stdout | 1 балл |
| Работа без интернета и загрузки внешних зависимостей | 1 балл |

Требования к образу: Ubuntu 22.04 LTS, ROS 2 Humble, `rmw_cyclonedds_cpp`,
только CPU, запуск без прав root, всё содержимое внутри образа.

## Состав проекта

- `Dockerfile` — образ на базе организаторского
  `registry.cookcloud.ru/base_images/ros2-base-humble-amd64:v1.0.1`
  (в Dockerfile запинен по digest `sha256:4d4a1849…eec243d`;
  Ubuntu 22.04 + ROS 2 Humble + CycloneDDS + пользователь `ros`, UID 1000);
- `entrypoint.sh` — контракт запуска: аргументы `ping`, `T2`, `T3`, `T4`, `T5`;
- `check_t1.sh` — локальная проверка образа по всем критериям ТЗ;
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

## Сдача решения (когда будут учётные данные)

1. `git push` — исходники в публичный GitHub-репозиторий, зафиксировать коммит:
   `git rev-parse HEAD`.
2. `docker login registry.cookcloud.ru` (учётка команды от организаторов).
3. Публикация:
   ```bash
   IMAGE="registry.cookcloud.ru/go1-hackathon-remote-stage-perspektiva/perspektiva:latest"
   docker buildx build --platform linux/amd64 -t "$IMAGE" --push .
   ```
4. Получить digest: `docker buildx imagetools inspect "$IMAGE"` → `Digest: sha256:…`
5. Проверить загрузку: `docker pull --platform linux/amd64 <repo>@<digest>`
6. В личный кабинет: ссылка на коммит GitHub + команда `docker pull` по digest.

Перед пересдачей при необходимости обновить `supported_tasks` в `entrypoint.sh`,
пересобрать и опубликовать — в кабинет вносится **новый digest**.
