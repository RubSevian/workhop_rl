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

Полные актуальные команды: [README запуска](../README.md). Старые пути из архивных инструкций не используются. Уже запущенный процесс продолжает работу с ранее загруженными значениями; перенос файла не меняет его состояние. При следующем leg launch нужно использовать новый config_path.

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
│   └── PROJECT_STRUCTURE.md                 архитектура и контракты
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
| Запуск и текущие параметры | `README.md` проекта |

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

Дополнительные README нашей Sim2Real реализации удалены; запуск, gains, CSV и deployment описаны в repos/workhop_rl/README.md. Отдельные отчёты и задания удалены; актуальная информация сведена в README.md и этот файл. Upstream README зависимостей сохранены.

## Контракт RL и обмен данными

Actor получает 5 кадров по63 значения, oldest→newest, tensor1×315. Кадр: gyro×0,25 [0:3]; projected gravity [3:6]; vx/vy/wz×[2,2,0,25] [6:9]; (leg q−default) [9:21]; leg dq×0,05 [21:33]; previous clipped action [33:45]; arm q6 [45:51]; arm dq6×0,05 [51:57]; accepted arm target6 [57:63]. Frame clip100. IMU quaternion нормализуется; projected gravity вычисляется inverse rotation.

Policy order FL/FR/RL/RR, hardware order FR/FL/RR/RL; перестановка12 индексов [3,4,5,0,1,2,9,10,11,6,7,8]. Action12 clipped100, target=default+0,25×action, конечный clamp±3,5рад. Default actor [0,1;0,8;−1,5] для FL/RL и [−0,1;0,8;−1,5] для FR/RR. Reset previous action/history выполняется при входе в RL, а не при RL_ZERO↔RL_ACTIVE. Нулевая команда скорости не означает нулевой action.

Worker50Гц запускает Torch CPU inference под thread-local InferenceMode; IO публикует500Гц. Worker использует captured snapshot; result принимается только при актуальном ticket/generation и выполнении deadline/freshness gates. Изменение command инвалидирует pending generation. Accepted result и pending job имеют отдельные40мс watchdogs; X handoff отдельно ограничен40мс и удерживает last target до accepted zero-result. Arm feedback/target проверяются отдельно.

CSV: kind0 — опубликованный IO target/kp/kd, q/dq, IMU, sticks/requested/command и ages; kind1 — completed policy, accepted, clipped action, candidate target, compute/job/captured ages. Массивы CSV в hardware order; quat xyzw. Sampling IO~50Гц, min/max интервалов собираются на каждом publish. Фиксированная очередь256/try_lock и отдельный writer; enabled/duration/path immutable startup flags, автоматический stop от первого sample и Trigger stop. Disabled logger не создаёт writer. Rejected candidate не означает публикацию. PASSIVE targets содержат stop sentinels и не являются углами.

## Текущее состояние физических проверок

Operator profile RL20/1,1, fixed40/1. Два A/X цикла прошли штатно. Последний прогон с CSV завершился policy_result_stale: accepted result age41,858мс, pending age22,377мс; текущий job compute22,488мс вернулся после fault и отвергнут. Затем Sport-status age500,850мс → Released=false → custom outputOFF. Это не доказательство повторного включения Sport.

CSV120с завершился примерно за10,5с доfault. Записано5495 policy results, все accepted, compute max18,757мс в записанном участке. Published target скачет до1,547рад между~20мс IO samples;119 скачков>.5рад внутри RL_ACTIVE,66 при равной command на границах samples. Policy rows также подтверждают большие изменения выхода при неизменной скорости. Это изменение q_des, не фактический мгновенный поворот сустава. Причина actor spikes не установлена; изменение gains не устранило скачки. Raw log/CSV остаются в runtime и ~/.ros, отдельно в Git не добавляются.

Functional regression33/33 PASS; build/read-only smoke PASS. Известный CPU timing FAIL остаётся незакрытым. Удаление документации не меняет gates, policy, режимы или физические процессы.
