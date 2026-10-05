# Go2 + RARS01 — System FSM Phase B.1

Jetson Orin Nano · ROS 2 Jazzy. Результаты и ограничения: [отчёт B.1](CODEX_SYSTEM_FSM_PHASE_B1_REPORT.md); [история Phase B](CODEX_SYSTEM_FSM_PHASE_B_REPORT.md). История решений: [CODEX_SIM2REAL_DECISIONS.md](CODEX_SIM2REAL_DECISIONS.md).

## Сборка и окружение

Candidate установлен отдельно в `install_fsm`. Работающий `install_r1` не заменён. Новые executable и arm RPC используются только после явного перезапуска соответствующего процесса оператором.

В каждом терминале candidate:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /home/ruben/go2_diploma/sim2real/setup.bash
source /home/ruben/go2_diploma/sim2real/install_fsm/local_setup.bash
```

Reference/Humble не используется как runtime. Для физического Ethernet оператор сохраняет прежние domain/RMW/CycloneDDS settings и `enP8p1s0`; для offline read-only достаточно loopback.

## Профили запуска

`operation_profile` задаётся только при старте. Изменение параметров, сервисы и кнопки не повышают capabilities.

| Профиль | A / ноги | Скорость |
|---|---|---|
| `read_only` | Диагностический intent; без Sport release/LowCmd | Нет |
| `arm_test` | Leg takeover запрещён; bench через единственный arm owner | Нет |
| `leg_safety_test` | Только захват измеренной позы и fixed hold | Нет |
| `rl_zero_test` | Прежний полный подъём и RL | Всегда `[0,0,0]` |
| `remote_test` | Прежний полный подъём и RL | Стики |
| `nav_test` | Подъём/RL после готовности NAV adapter | `/cmd_vel` |
| `full_mission` | Дополнительно нужны perception и arm emergency validation | NAV/mission |

Неизвестное имя или конфликт явного profile с legacy flags завершают startup до физических side effects. Legacy launch без profile сохраняет отображение: read_only → `read_only`; motion=false → `rl_zero_test`; remote+motion → `remote_test`; autonomy+motion → `nav_test`.

## Read-only проверка

```bash
ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  operation_profile:=read_only \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt
```

В другом терминале:

```bash
ros2 run unitree_legged_real r3_status --once
```

`state` — один из INIT/STANDBY/TAKEOVER/ACTIVE/CONTROLLED_STOP/SYSTEM_HOLD/EMERGENCY_FAULT. `phase` показывает этап; `legacy_state` оставлен для прежних диагностических инструментов. Проверяйте `operation_profile`, `arm_control_ready`, `arm_home_ready`, `stop_blocker` и состояния async ports.

## Физический RL-профиль — команда для оператора

Прежний тестовый YAML сохраняет операторские trial gates. Они не доказывают physical PASS новой автоматики X/B.

```bash
ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  operation_profile:=rl_zero_test \
  config_path:=/home/ruben/go2_diploma/sim2real/runtime/r3_first_rl_zero.yaml \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt \
  network_interface:=enP8p1s0
```

Launch сам не создаёт LowCmd. До A нужны свежие LowState/remote/Sport, HOME и остальные gates. A: Sport release при необходимости → lease/graph verification → measured hold ≥0,02 с → линейный stand 6 с → fixed hold 4 с → policy reset → ACTIVE/phase RL_ZERO. Fixed gains 40/1; после первого policy результата — RL 25/1. Скорость zero не означает нулевые суставные actions.

## Пульт

Удерживать комбинацию ≥0,75 с, затем отпустить. Приоритет B > X > A.

| Кнопки | Действие |
|---|---|
| **L1+L2+A** | Takeover и прежний цикл до RL; из лежащего SYSTEM_HOLD — заново lease/publisher, fresh measured capture, без повторного release при fresh RELEASED |
| **L1+L2+X** | Из ACTIVE: закрыть скорости → zero RL → HOME через owner → fresh measured PD handoff → lie-down → reached/settle → output OFF, publisher/lease освобождены → SYSTEM_HOLD |
| **L1+L2+B** | Защёлкнуть emergency; ноги 0/3 при прежней eligibility; отменить миссии; запросить отдельно валидированный arm disable; без HOME wait и auto recovery |

**Новая динамика lie-down пока не commissioned.** `lie_down.operator_validated=false`: после HOME X сохраняет zero RL и показывает `lie_down_dynamics_not_commissioned`; он не выключает RL и не начинает траекторию. Для явно разрешённого commissioning trial добавьте при запуске `controlled_stop_lie_down_trial:=true`; production YAML и validation flags остаются false. Параметр неизменяемый, статус показывает trial и `lie_down_dynamics_validated=false`. Approved target FR/FL/RR/RL:

```text
[0.01,1.30,-2.70, -0.01,1.30,-2.70, -0.30,1.30,-2.70, 0.30,1.30,-2.70]
```

Duration 8 с, fixed gains 40/1, tolerance 0,15 рад, timeout 12 с и settle 0,2 с — commissioning candidates. Для проверки используйте явный trial-флаг; `operator_validated` меняется только после подтверждения динамики. Успешный X отключает custom output после непрерывного reached/settle; электрическое отключение моторов этим не подтверждается. Повторный X в SYSTEM_HOLD ничего не включает. HOME timeout сохраняет zero RL. Lie timeout сохраняет последний planned fixed target и LowCmd. Critical invalid/stale/policy fault переводит в EMERGENCY_FAULT.

**B не задаёт lie-down trajectory.** Arm emergency gate по умолчанию false; принятый service reply не подтверждает физическое отключение. Ctrl+C завершает процесс и не заменяет B.

## Рука

Один serial owner; SDK GUI и второй owner одновременно не запускать. Уже работающий старый owner не получает новые RPC от пересборки. Новые HOME/emergency endpoints появляются при явном запуске candidate owner.

```bash
ros2 run unitree_legged_real rars_r3_owner --ros-args \
  -p read_only:=false -p connect_serial:=true \
  -p sdk_config_path:=/home/ruben/go2_diploma/sim2real/repos/rars01_graspnet/config/default.yaml \
  -p config_path:=/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml \
  -p device_path:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3172366B3233-if00 \
  -p lock_directory:=/home/ruben/go2_diploma/sim2real/runtime/rars_serial_leases \
  -p journal_path:=/home/ruben/go2_diploma/sim2real/runtime/rars_auto_home/enable-journal
```

Сохраняются connect →10 с→enable один раз→семь нулевых HOME targets 100 Гц. Feedback появляется после enable. Calibration и serial leases прежние; journal только диагностический. Runtime fault останавливает текущую сессию без auto re-enable.

`/rars01/control/return_home` принимает idempotent HOME intent только у healthy enabled owner, не вызывает enable. HOME completion проверяется по реальным свежим q/dq/accepted target. `/rars01/control/emergency_disable` запрещён, пока отдельный `rars01.emergency_disable_validated=false`; leg gains на руку не копируются.

## Стики и NAV

Для стиков выберите `operation_profile:=remote_test`: `ly` ±0,20 м/с вперёд, `-rx` ±0,10 м/с вбок, `-lx` ±0,10 рад/с поворот; deadband 0,01. Нейтраль оставляет RL с zero velocity.

Для NAV выберите `nav_test`. Сохраняются TwistStamped `/cmd_vel`, QoS1, finite/header/receive freshness ≤0,25 с и clamp ±0,20/0,10/0,10. `pathFollower.sendSportCommand=false`. Stale NAV даёт zero velocity, без переключения профиля.

Факт `/cmd_vel` не даёт navigation_ready. Реальный adapter должен публиковать `/go2/mission/readiness` с `observed_ns` (steady-clock ns), непустым `session_id`, `navigation_ready`, `perception_ready`, и предоставлять Trigger `/go2/mission/cancel_navigation`; manipulation cancellation — `/rars01/mission/cancel_manipulation`. Readiness TTL 0,5 с. Пока adapters отсутствуют, NAV/FULL блокируются; cancellation отображается unavailable, не PASS. Эти adapters и search/grasp controller не создаются данным FSM refactor.

## Диагностика

```bash
ros2 topic echo /go2/locomotion_status
ros2 run unitree_legged_real go2_mode_switch --interface enP8p1s0 --status
```

Watchdog ответа и незавершённого job — прежние 40 мс; captured arm inputs — отдельно 0,25 с. При fault сохраняйте `state`, `phase`, `fault`, `policy_ms`, policy ages и transition log. Старт нового процесса не обновляет уже работающий launch.

## Явный X trial и повторный A

После RL_ZERO smoke и проверки стиков команду запуска ног оператор меняет на:

```bash
ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  operation_profile:=remote_test \
  controlled_stop_lie_down_trial:=true \
  config_path:=/home/ruben/go2_diploma/sim2real/runtime/r3_first_rl_zero.yaml \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt \
  network_interface:=enP8p1s0
```

HOME owner запускается отдельно командой выше. A → прежний подъём/RL; X → zero RL до HOME → measured PD → lie-down/settle → SYSTEM_HOLD. Ожидаемые поля: `custom_leg_output: OFF`, `output_enabled: false`, `lowcmd_publisher_present: false`, `lowcmd_lease_present: false`, `output_stop_confirmed: true`; sent_packets больше не растёт. Sport автоматически не включается. Новый A проверяет отсутствие чужих publishers/lease и снова захватывает свежие измеренные углы. Нейтраль стиков даёт zero velocity; REMOTE не слушает NAV.

## Отдельная IMU calibration для NAV

Существующая `autonomy_nav_go2:ros2_Jazzy`, package/executable `calibrate_imu`, управляет Sport API `/api/sport/request` и публикует `/cmd_vel`. Это отдельный Sport-mode тест, не профиль System FSM. Запускайте только при остановленном custom controller/LowCmd OFF и вручную подтверждённом штатном Sport Mode. SYSTEM_HOLD после X оставляет Sport RELEASED и сам по себе этому условию не соответствует. Рука может оставаться HOME.

В терминале сборки navigation workspace с установленным `calibrate_imu`:

```bash
source install/setup.bash
ros2 run calibrate_imu calibrate_imu
```

~2 с zero motion, static bias до15 с, затем положительное вращение Z 1,396 рад/с до35 с, StopMove и запись `~/Desktop/imu_calib_data.yaml`. `transform_everything.py` ожидает именно этот путь. После завершения проверьте файл и перезапустите NAV. Этот utility в рамках B.1 не запускался и не переносился на RL.
