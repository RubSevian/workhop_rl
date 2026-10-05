# Go2 + RARS01 — Sim2Real на Jazzy

Рабочая ветка: `ros2_go2_rars01_real`. Среда: Jetson Orin Nano, ROS 2 Jazzy.

- [Запуск: окружение, рука, нулевой тест RL, пульт и навигация](docs/sim2real/README.md)
- [Актуальные решения, параметры и результаты проверок](docs/sim2real/CODEX_SIM2REAL_DECISIONS.md)
- [Файлы deployment AUTO HOME](src/unitree_ros2_to_real/deployment/README.md)

Версионные копии README и единого отчёта находятся в `docs/sim2real/`; рабочие копии — в корне `sim2real/`. Конфигурация ног — `src/unitree_ros2_to_real/config/go2_rars01_real.yaml`; отдельный операторский trial profile — `sim2real/runtime/r3_first_rl_zero.yaml`.

Исходные задания фаз сохранены в `docs/`. Они описывают требования прошлых этапов; текущий порядок запуска задаёт README workspace. Sim2Sim reference остаётся read-only, его Humble build/install не используется на Jetson.
