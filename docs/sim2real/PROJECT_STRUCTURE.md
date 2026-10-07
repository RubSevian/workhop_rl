# Структура Sim2Real и размещение конфигураций

Актуально на07.10.2026 после переноса YAML. Рабочая среда: Jetson Orin Nano, ROS2 Jazzy. Изменены размещение файлов и ссылки; команды роботу не отправлялись, работающие процессы не перезапускались.

## Что исправлено

Операторский YAML хранился одновременно среди runtime outputs и как deployment snapshot. Теперь **единственный редактируемый профиль находится в ROS package config/**:

```text
/home/ruben/go2_diploma/sim2real/repos/workhop_rl/
└── src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml
```

Переименован из r3_first_rl_zero.yaml: профиль уже используется для commissioning/REMOTE walking, а operation_profile (remote_test/rl_zero_test/…) выбирается отдельно launch argument. Содержимое перенесено побайтово, без изменения gains, gate flags, poses, commands limits или actor contract.

SHA256 до/после: `73c9d8f09e68818478bd43cdcb5739f79bfc794cc77d268b98cb63e2ada70ee5`.

Удалены прежние копии в workspace runtime/, deployment/workspace_runtime/ и устаревшая generated копия install_fsm/deployment/workspace_runtime/. В source остаётся один commissioning YAML; installed config — автоматически создаваемый результат сборки, не второй редактируемый источник.

Новая строка в запуске:

```bash
config_path:=/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml
```

Полные актуальные команды: [README запуска](../../README.md). Старые пути из архивных инструкций не используются. Уже запущенный процесс продолжает работу с ранее загруженными значениями; перенос файла не меняет его состояние. При следующем leg launch нужно использовать новый config_path.

## Реальная структура workspace сейчас

```text
/home/ruben/go2_diploma/
├── sim2sim_reference/go2_diploma_sim2sim/     read-only reference
└── sim2real/
    ├── setup.bash                          только настройка окружения Jazzy
    ├── repos/                              исходники отдельных Git repositories
    │   ├── workhop_rl/                     RL deployment, System FSM, ROS node ног
    │   ├── autonomy_nav_go2/                navigation/IMU/SLAM/command source
    │   ├── rars01_graspnet/                 arm/perception config и grasp integration
    │   ├── rars_arm_sdk/                    serial SDK/протокол STM32
    │   └── rars01_description/              URDF/meshes/robot description
    ├── weights/
    │   ├── policy_2.pt                     выбранный TorchScript actor
    │   └── POLICY_MANIFEST.yaml             hash/contract/provenance
    ├── scripts/                            workspace build/test/verify helpers
    ├── docs/archive/                       старые корневые задания и копии отчётов
    │   └── probes/                         исторические capture/release probe scripts
    ├── runtime/                            результаты запусков и состояние процессов
    │   ├── *.log, *.csv, *.json              test/diagnostic outputs
    │   ├── status*.yaml                    YAML snapshots статуса, не launch config
    │   ├── ros_logs/, *_ros_logs/           вывод ROS проверок
    │   ├── lie_pose_capture/, rl_joint_audit/ measurements/reports
    │   ├── rars_auto_home/                 arm enable/fault journal
    │   ├── rars_serial_leases/             владение serial port
    │   └── go2_lowcmd.lock                 владение leg output
    ├── build_fsm/                          CMake objects/tests текущего leg package
    ├── install_fsm/                        текущий executable/installed configs
    ├── log_fsm/                            colcon build logs
    ├── build_r1/, install_r1/              базовая Jazzy сборка/dependencies
    └── log_r1/                             её colcon logs
```

`install_r1` пока нужен окружению как underlay для сообщений Unitree и RL libraries; его не удаляли. `install_fsm` добавляется поверх него для текущего controller. Source setup.bash и install_fsm/local_setup.bash настраивает окружение, не включает моторы. Reference Humble install/build не используется.

`runtime/status*.yaml` и журналы остаются: это результаты измерений и runtime state, а не настройки старта. Leases/journals не переносились и не очищались. Исторические probes убраны из runtime в архив, не выполнялись; их прежние относительные пути сохранены как история, это не актуальные entrypoints.

## Структура основного проекта workhop_rl

```text
repos/workhop_rl/
├── README.md                               единственная инструкция запуска
├── jazzy_setup.sh                          проверка/подключение Jazzy underlay
├── docs/
│   ├── sim2real/                           decisions и технические отчёты
│   │   ├── PROJECT_STRUCTURE.md            этот документ
│   │   ├── CODEX_SIM2REAL_DECISIONS.md      единый журнал решений
│   │   └── CODEX_TEMP_MOTION_LOGGING_AND_GAINS.md
│   ├── sim2real_r1/                        документы ранних этапов
│   └── sim2real_r3/                        задания/документы R3
└── src/
    ├── unitree_ros2_to_real/                ROS package unitree_legged_real
    │   ├── config/
    │   │   ├── go2_rars01_real.yaml         fail-closed/default deployment
    │   │   ├── profiles/
    │   │   │   └── go2_rars01_commissioning.yaml  выбранный операторский профиль
    │   │   └── прочие retained sim/RViz configs
    │   ├── launch/                         ROS launch descriptions
    │   ├── src/
    │   │   ├── go2_r3_commissioning.cpp     ROS inputs, worker50Гц, output500Гц
    │   │   ├── r3_commissioning.cpp         System FSM, X/A/B, watchdogs
    │   │   ├── real_controller_core.cpp    actor загрузка/перестановка суставов
    │   │   ├── safety_io.cpp               feedback/remote decode, LowCmd/CRC
    │   │   ├── rars_r3_owner.cpp           один owner руки, HOME/feedback/ports
    │   │   └── motion_trace.cpp            временный асинхронный CSV logger
    │   ├── include/                        contracts/types/interfaces
    │   ├── tests/                          offline regression/contract tests
    │   ├── deployment/                     service/env/startup helpers
    │   │   ├── workspace_scripts/          versioned snapshot workspace helpers
    │   │   └── workspace_setup.bash         source для workspace setup
    │   ├── library/unitree_sdk2/            SDK2/DDS/MotionSwitcher dependency
    │   ├── unitree_legged_sdk/              retained legacy SDK
    │   └── CMakeLists.txt, package.xml
    ├── unitree_rl_controller-ros2/          Agent, observation63/history315/action12
    ├── ros2_unitree_legged_msgs/            ROS message definitions
    ├── unitree_ros2/                       retained DDS/integration sources
    └── unitree_mujoco/                     retained simulator assets/code
```

Реальная сборка не включает legacy MuJoCo executable по умолчанию. Legacy files сохранены для reference/parity; они не являются runtime на Jetson.

## Что где редактировать

| Что | Где |
| --- | --- |
| Текущий операторский запуск, gains/limits/pose/gates | `src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml` |
| Default fail-closed deployment | `src/unitree_ros2_to_real/config/go2_rars01_real.yaml` |
| RL gains | `go2_rars01.rl_kp`, `go2_rars01.rl_kd` в выбранном config_path |
| Stand/fixed hold/штатный lie-down gains | `go2_rars01.fixed_kp`, `go2_rars01.fixed_kd` в том же YAML |
| Профиль возможностей и source команд | launch `operation_profile`, отдельный от YAML parameter |
| Временные логи движения | launch motion_diagnostics_*; CSV выходит в runtime/ |
| Policy/scales/history | source Agent/observation contract; веса в weights/ |
| Порядок переходов/защита | source R3Supervisor/ROS node |
| Инструкции и текущие решения | `docs/sim2real/` проекта |

Текущие gains сохранены: RL25/1, fixed40/1. Fixed gains общие для подъёма/hold/lie-down. Все массивы по12 значений. Правка source YAML действует после нового запуска controller и не требует компиляции C++; installed YAML обновляется сборкой и вручную не редактируется.

## Проверки переноса

- Runtime/source копии перед переносом совпадали; новый source YAML и installed config совпадают побайтово.
- Candidate colcon build/install PASS; профиль установлен в share/unitree_legged_real/config/profiles/.
- Launch/lifecycle tests2/2 PASS; verify_r1/r2/r3 PASS; git diff --check PASS.
- Smoke helper пишет node log в runtime/, а не в корень workspace; source snapshot helper обновлён вместе с рабочим скриптом.
- Новые рабочие команды в документах проекта не ссылаются на удалённый runtime YAML. Архивные копии сохранены неизменными.
- Actor/gains/pose/gates/Sport/serial state не изменялись. Физический запуск агентом не выполнялся.
- CPU timing в рамках переноса не тестировался; ранее незакрытый timing вопрос сохраняется.

Логи: runtime/project_structure_build.log, project_structure_tests.log, project_structure_verify.log, config_relocation_sha256.txt. Перенос структуры и предыдущая временная диагностика пока не закоммичены.

## Объединение README

Дополнительные README нашей Sim2Real реализации удалены; запуск, gains, CSV и deployment описаны в repos/workhop_rl/README.md. Аудит данных сохранён как CODEX_RL_DATA_AND_MOTION_AUDIT.md. Upstream README зависимостей сохранены.
