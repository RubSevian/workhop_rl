# System FSM — PHASE B.1: lie-down → output OFF

05.10.2026. Требования: [CODEX_SYSTEM_FSM_PHASE_B1_LIEDOWN_OUTPUT_OFF.md](CODEX_SYSTEM_FSM_PHASE_B1_LIEDOWN_OUTPUT_OFF.md). База: `e94afad`, ветка `ros2_go2_rars01_real`.

## Что изменено

**X после успешного опускания выключает custom leg output и освобождает publisher/lease. A из этого SYSTEM_HOLD заново получает output с текущими измеренными углами.** Семь глобальных состояний и единственный владелец FSM сохранены.

| Комбинация | Последовательность |
|---|---|
| L1+L2+A | Прежний Sport takeover → measured hold0,02 с → stand6 с → hold4 с → reset → RL. Из лежащего SYSTEM_HOLD: fresh RELEASED → проверка чужих publishers/lease → новый publisher → fresh measured capture → тот же подъём/RL; без повторного Sport release |
| L1+L2+X | Закрыть скорости, zero RL → cancel NAV/manip → ARM HOME → свежий HOME и settle → measured-q PD handoff → плавное lie-down → reached и непрерывный settle → прекратить публикацию → удалить publisher → освободить lease → подтверждение → SYSTEM_HOLD |
| L1+L2+B | Прежняя центральная emergency, приоритет над X/A; legs0/3 при eligibility, arm disable отдельно gated; без автоматического восстановления |

До HOME RL продолжает работать. Первый PD packet после HOME равен свежему measured_q, без скачка к stand/lie target. При успешном reached/settle `Tick` возвращает отсутствие packet; нулевой gains packet вместо shutdown не отправляется. Внутренняя `LIE_DOWN_OUTPUT_STOPPING` остаётся CONTROLLED_STOP, пока существующий `StopPublisher` не удалит publisher/lease и не подтвердит завершение. Только затем SYSTEM_HOLD.

Повторный X в лежащем SYSTEM_HOLD идемпотентен. Старый pending policy отвергается; orchestration generation изменяется при shutdown и restart, старые HOME/cancel callbacks не могут возобновить работу. SYSTEM_HOLD после X не включает Sport и не запускает другой controller. LEG_SAFETY captured hold сохраняет своё прежнее live-output поведение.

## Trial и параметры

Production и operator YAML сохраняют `lie_down.operator_validated=false`, `controlled_stop.lie_down_trial=false`. Для разрешённого физического trial оператор явно задаёт launch-параметр **`controlled_stop_lie_down_trial:=true`**. Он read-only после startup, не меняет OperationProfile и не разрешает чужую позу. В YAML эквивалент — `real_deployment.r3_commissioning.controlled_stop.lie_down_trial: true` в отдельной явной trial-конфигурации.

Approved target, hardware FR/FL/RR/RL:

```text
[0.01,1.30,-2.70, -0.01,1.30,-2.70, -0.30,1.30,-2.70, 0.30,1.30,-2.70]
```

| Параметр | Значение в candidate |
|---|---|
| Capture / stand / stand hold | 0,02 / 6 / 4 с, прежняя линейная интерполяция |
| Fixed / RL gains | Kp40/Kd1 / Kp25/Kd1 |
| Lie-down duration | 8 с |
| Reached tolerance / timeout / settle | 0,15 рад / 12 с / 0,2 с |
| HOME timeout / settle | 10 с / 0,2 с |

**Траектория и её критерии остаются commissioning candidates, physical PASS не заявлен.** Target approved не означает dynamics validated. При HOME timeout здоровый zero RL остаётся активен. При lie-down timeout или потере reached до завершения settle output остаётся ON, fixed PD удерживает последний planned target, status показывает LIE_DOWN_BLOCKED. Critical input/policy/ownership fault остаётся центральной emergency.

## Статус после успешного X

```yaml
state: SYSTEM_HOLD
phase: LIE_DOWN_HOLD
custom_leg_output: "OFF"
output_enabled: false
lowcmd_publisher_present: false
lowcmd_lease_present: false
output_stop_confirmed: true
motor_power_off_confirmed: false
sport_state: RELEASED
```

`sent_packets` больше не растёт. `controlled_stop_lie_down_trial` показывает разрешение trial, `lie_down_commissioning_active` — выполнение нового stop в trial, `lie_down_dynamics_validated` остаётся false. Custom output OFF подтверждает прекращение программной публикации и освобождение своего ownership; электрическое отключение моторов/поведение firmware после последнего packet не измерялось.

## Проверки

**PASS: candidate Release/Jazzy/aarch64 сборка; 28/28 regression tests; baseline A byte-identical; ROS read-only smoke; verify_r1/r2/r3.**

| Лог в `sim2real/runtime/` | Результат |
|---|---|
| `fsm_stepb1_build.log` | Согласованная сборка/установка `build_fsm/install_fsm` |
| `fsm_stepb1_test.log` | 16/16 targeted tests PASS после финальных исправлений |
| `fsm_b1_full_regression.log` | 28/28 PASS, CPU benchmark исключён; actor/CRC/SDK/RARS/новый X/A/B включены |
| `fsm_b1_readonly_ros_smoke.log` | Loopback domain223, read_only=true и trial=true: строковый OFF/status; immutable trial/profile; NAV clamp/freshness; X/B; sent_packets0 и zero LowCmd publishers |
| `fsm_b1_verify.log` | verify_r1/r2/r3 PASS, production defaults и source isolation |

ROS trial параметр был задан при startup, его runtime изменение отклонено. Smoke не запускал arm owner и не использовал физический Ethernet/SDK. Все физические lie-down/restart проверки остаются НЕ ВЫПОЛНЕНЫ.

Начальная ROS smoke выявила проблему сериализации: некавыченный ON/OFF YAML1.1 parser воспринимал как boolean. Вывод исправлен на строки с явными кавычками; исходный отказ сохранён в `runtime/fsm_b1_readonly_ros_smoke_initial_failed.log`. Это исправление статуса, не изменение команд управления.

Новые тесты проверяют полный X, ожидание transport acknowledgement, реальные file-lock unlock/reacquire на временном `/tmp` lease, свежий первый target после restart, continuous settle/dropout, timeout с output ON, trial/default gates и недопустимую чужую позу. Publisher в lifecycle fixture — mock, **не физический DDS publisher**. Порядок существующего node lifecycle и foreign-publisher gates дополнительно проверен по runtime source. Физическое достигнутое положение и firmware motor power-off этим не подтверждены.

Baseline actor315-D/mapping/history, 500Гц IO/50Гц policy, tickets/watchdogs, packet/CRC, Sport helper, RARS owner/calibration и NAV callback clamp/finite/freshness сохранены. CPU benchmark в B.1 не повторялся: compute path не менялся; предыдущие измерения находятся в отчёте Phase B и не заменяют длительную нагрузку всей системы.

## Запуск оператором

Candidate: `install_fsm`; `install_r1` не заменён. Подробные команды запуска HOME owner и всех профилей: [README](../../README.md).

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /home/ruben/go2_diploma/sim2real/setup.bash
source /home/ruben/go2_diploma/sim2real/install_fsm/local_setup.bash

ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  operation_profile:=remote_test \
  controlled_stop_lie_down_trial:=true \
  config_path:=/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt \
  network_interface:=enP8p1s0
```

Порядок commissioning: сначала `operation_profile:=rl_zero_test` smoke; затем remote_test с небольшими командами; X trial; проверка output OFF; A из лежащего SYSTEM_HOLD; несколько A/X циклов. Стики REMOTE сохранены: vx±0,20, vy±0,10, wz±0,10, нейтраль даёт `[0,0,0]`. REMOTE не получает команды NAV. Физические наблюдения записываются отдельно от synthetic PASS.

## IMU calibration

Проверен существующий код `autonomy_nav_go2:ros2_Jazzy`, `src/utilities/calibrate_imu/src/calibrate_imu.cpp`: `/utlidar/imu`, прямой Sport API `/api/sport/request` и `/cmd_vel`; zero первые2 с, static bias до15 с, положительное Z вращение1,396 рад/с с15 до35 с, StopMove и запись `~/Desktop/imu_calib_data.yaml`. `transform_sensors/transform_everything.py` загружает этот точный путь.

Запуск отдельный, из терминала установленного navigation workspace:

```bash
source install/setup.bash
ros2 run calibrate_imu calibrate_imu
```

Custom controller должен быть остановлен/LowCmd OFF, штатный Sport Mode подтверждён вручную; рука может оставаться HOME. SYSTEM_HOLD после X оставляет Sport RELEASED, поэтому не равен готовности к Sport calibration. После калибровки проверить файл и перезапустить NAV. Утилита не запускалась, navigation repo не изменён и её сборка не включалась в B.1.

## Файлы и сохранение

В `repos/workhop_rl/src/unitree_ros2_to_real/`: `include/r3_commissioning.hpp`, `include/system_state.hpp`, `src/r3_commissioning.cpp`, `src/system_state.cpp`, `src/go2_r3_commissioning.cpp`, launch, production/trial YAML, CMake; новые `system_output_off_test.cpp` и `system_output_lifecycle_test.py`; обновлены controlled-stop/restart/emergency/launch tests и workspace read-only smoke. README, specification и этот отчёт сохранены в `docs/sim2real/`; workspace copies README/trial YAML/smoke/decision journal синхронизированы.

Изменения B.1 пока **не закоммичены и не запушены**. Агент не отправлял реальный LowCmd, Sport release/enable, serial enable/targets/disable или команды движения; не останавливал физические процессы. Новая физическая X/A динамика, disable руки и NAV/FULL остаются отдельными commissioning этапами.
