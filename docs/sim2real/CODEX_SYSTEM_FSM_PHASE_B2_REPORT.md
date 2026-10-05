# System FSM — PHASE B.2: explicit PASSIVE

06.10.2026. Требования: [CODEX_SYSTEM_FSM_PHASE_B2_PASSIVE_MODE.md](CODEX_SYSTEM_FSM_PHASE_B2_PASSIVE_MODE.md). Ветка `ros2_go2_rars01_real`.

## Сохранение предыдущих изменений

**B.1 закоммичен до начала B.2: `e2990c6` — `feat: release leg output after controlled lie-down and reacquire on restart`.** Включены code/tests/config/README/отчёт B.1. Изменения не пушились. Отметка «не закоммичены» в отчёте B.1 описывает его состояние до этого commit.

## Новое поведение

**Успешный X после lie-down/reached/settle отправляет короткую серию PASSIVE, затем освобождает publisher/lease. Новый A восстанавливает mode1 с текущими измеренными углами. B остаётся emergency damping0/3.**

```text
L1+L2+X
→ zero velocity, RL продолжает работать
→ cancel NAV/manip, ARM HOME, fresh HOME/settle
→ fresh measured-q handoff в fixed PD
→ smooth approved lie-down, reached и непрерывный settle
→ LIE_DOWN_PASSIVE: 10 packets через существующий IO500Гц
→ прекратить публикацию, удалить publisher, освободить lease
→ OUTPUT_STOPPED acknowledgement
→ SYSTEM_HOLD
```

Для всех12 ног PASSIVE packet: mode0, kp0, kd0, tau0, q=`PosStopF`2,146E9, dq=`VelStopF`16000. Sentinels берутся из уже существующего MakeLowCmd; layout/CRC рассчитываются прежними SerializeLowCmd/Go2Crc. Индексы12..19 не изменяются. Вендорский пример [low_level_ctrl.cpp](../../src/unitree_ros2/example/src/src/low_level_ctrl.cpp) различает mode1 и passive0; Go2 SDK example использует те же sentinels. Это основание программной реализации, не измерение firmware на этом роботе.

Серия: **ровно10 успешных локальных publish**, номинально около20 мс. Счётчик растёт только после возврата `output_->publish`; генерация packet не выдаётся за отправку. Невалидный CRC и повтор acknowledgement не увеличивают счётчик. Если серия не завершилась за100 мс, explicit `passive_transition_timeout` ведёт в прежнюю central emergency; бесконечный passive publisher не остаётся.

PASSIVE допускается только внутри LIE_DOWN_PASSIVE после успешного controlled lie-down. Packet validator сравнивает весь canonical packet, включая sentinels/modes/reserves/CRC. Обычный active validator не ослаблен. Ни HOME timeout, ни lie timeout, ни потеря reached до завершения settle не запускают успешный passive shutdown. Critical fault/B отменяют нормальный stop; B не превращён в passive.

После A из лежащего SYSTEM_HOLD: fresh RELEASED → foreign publishers/lease checks → reacquire → новый publisher → свежий measured_q; первый packet содержит mode1 для всех12 ног, q=measured_q, fixed gains40/1. Затем прежние capture0,02 с, stand6 с, hold4 с, один reset и RL25/1. Stored lie_down_q и stand target не используются как первый active target. Sport автоматически не включается.

## Статус успешного X

```yaml
state: SYSTEM_HOLD
phase: LIE_DOWN_HOLD
sport_state: RELEASED
custom_leg_output: "OFF"
output_enabled: false
lowcmd_publisher_present: false
lowcmd_lease_present: false
output_stop_confirmed: true
passive_command_sent: true
passive_packets_sent: 10
passive_sequence_complete: true
last_commanded_leg_mode: 0
motor_power_off_confirmed: false
```

`sent_packets` перестаёт расти. Passive sent/completed означают software publish, не hardware acknowledgement и не электрическое motor power-off. При новом enable начинается новый счёт серии; после первого active publish last_commanded_leg_mode=1. При прерванной серии sent может быть true, если часть packets действительно опубликована (включая запоздалый publish), но complete=false; состояние EMERGENCY_FAULT не выдаётся за успешный shutdown.

## Проверки

**Software PASS: сборка Release/Jazzy/aarch64, 30/30 tests, read-only ROS smoke, verify_r1/r2/r3 и candidate package resolution.**

| Лог в `sim2real/runtime/` | Результат |
|---|---|
| `fsm_stepb2_build.log` | Финальная согласованная сборка/установка install_fsm PASS |
| `fsm_stepb2_test.log` | 17/17 targeted PASS |
| `fsm_b2_full_regression.log` | 30/30 PASS; initial A byte-identical; actor/packet/CRC/SDK/RARS/NAV/X/A/B и CPU |
| `fsm_b2_readonly_ros_smoke.log` | Loopback223/read_only/trial: passive counters0 и mode unknown, output OFF, immutable parameters, NAV clamp/expiry, X/B, zero LowCmd publishers |
| `fsm_b2_verify.log` | verify_r1/r2/r3 PASS |
| `fsm_b2_install_check.log` | ROS package prefix именно install_fsm; executable/tools присутствуют |
| `fsm_b2_host_readiness.log` | Повторная host проверка: Ethernet NO-CARRIER, USB STM32 присутствует, leg/arm owner processes отсутствуют |

CPU benchmark: threads1, по50 warmup и500 samples. JIT mean0,771155/p99 0,83619/max0,893664 мс; полный agent mean1,21669/p99 1,34978/max1,36223 мс при бюджете20 мс. Это короткий synthetic compute test, не длительное измерение всей системы с LiDAR/NAV и физическим feedback.

Первый targeted прогон выявил ошибку synthetic dropout fixture: reached пропадал уже после завершения settle, поэтому PASSIVE был корректно разрешён. В fixture dropout перенесён до settle completion; исправление не меняло runtime criteria. Исходный отказ сохранён в `fsm_b2_fixture_initial_failed.log`; все последующие targeted и полная регрессия PASS. Дополнительно проверены B посреди passive серии и запоздалый publish: факт отправки записывается, но deadline violation не выдаётся за successful completion.

Физические LowCmd/passive/SDK Sport release/enable, serial enable/targets/disable и движения агентом **не отправлялись**. Физические процессы не останавливались. Тесты packet/FSM используют synthetic observations и mock transport; ROS smoke только read_only/loopback. Физическое расслабление ног, новая lie-down динамика и реакция firmware ещё не проверены.

## Готовность к физическому тесту

**На момент проверки Ethernet `enP8p1s0` — DOWN/NO-CARRIER. Немедленный тест связи с Go2 не готов.** STM32 serial path существует и указывает на ttyACM0; это подтверждает наличие USB device, а не свежий motor feedback/HOME. Host process check не обнаружил go2_r3_commissioning или rars_r3_owner. SDK status при отсутствии carrier не запускался.

Installed executable диагностики называется `r3_status.py` (с `.py`); команда без суффикса в README исправлена. go2_r3_commissioning, go2_mode_switch и rars_r3_owner присутствуют в candidate install.

Software candidate находится в отдельном `build_fsm/install_fsm`; `install_r1` не заменён. Production/trial defaults остаются false: `lie_down.operator_validated`, `controlled_stop.lie_down_trial` и arm emergency validation. Для commissioning используется явный immutable launch `controlled_stop_lie_down_trial:=true`. Approved target:

```text
FR FL RR RL:
[0.01,1.30,-2.70, -0.01,1.30,-2.70, -0.30,1.30,-2.70, 0.30,1.30,-2.70]
```

Lie duration8 с, fixed gains40/1, tolerance0,15 рад, timeout12 с, settle0,2 с; HOME timeout10 с/settle0,2 с. Это commissioning параметры, physical validation не повышалась. REMOTE bounds и NAV clamp/freshness сохранены, actor315-D/mapping/history, 500/50Гц scheduling и watchdogs не менялись.

## Команды оператору

Подключить Ethernet/включить Go2; в каждом терминале candidate сохранить прежние физические DDS/domain settings и выполнить:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /home/ruben/go2_diploma/sim2real/setup.bash
source /home/ruben/go2_diploma/sim2real/install_fsm/local_setup.bash

ip -brief link show enP8p1s0
ros2 run unitree_legged_real go2_mode_switch --interface enP8p1s0 --status
```

Нужно наличие carrier и успешный fresh Sport status. **Вручную выключать Sport для A не нужно:** первоначальный A сам запрашивает release; если Sport уже fresh RELEASED, release пропускается.

Отдельный терминал HOME owner, при отсутствии второго SDK GUI/owner:

```bash
ros2 run unitree_legged_real rars_r3_owner --ros-args \
  -p read_only:=false -p connect_serial:=true \
  -p sdk_config_path:=/home/ruben/go2_diploma/sim2real/repos/rars01_graspnet/config/default.yaml \
  -p config_path:=/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml \
  -p device_path:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3172366B3233-if00 \
  -p lock_directory:=/home/ruben/go2_diploma/sim2real/runtime/rars_serial_leases \
  -p journal_path:=/home/ruben/go2_diploma/sim2real/runtime/rars_auto_home/enable-journal
```

Прежний connect →10 с→enable→HOME; дождаться свежего arm_control_ready/arm_home_ready. Агент этот запуск не выполнял.

Первый запуск ног с zero velocity и явным X trial:

```bash
ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  operation_profile:=rl_zero_test \
  controlled_stop_lie_down_trial:=true \
  config_path:=/home/ruben/go2_diploma/sim2real/runtime/r3_first_rl_zero.yaml \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt \
  network_interface:=enP8p1s0
```

В другом терминале:

```bash
ros2 run unitree_legged_real r3_status.py --once
ros2 topic echo /go2/locomotion_status
```

A (L1+L2+A ≥0,75 с, отпустить) → stand → hold → RL zero. X (L1+L2+X ≥0,75 с) → HOME → lie-down/settle → PASSIVE10 → SYSTEM_HOLD/OFF. Проверить status выше и **физически** проверить расслабление ног. Следующий A в этом же процессе → measured active mode1 → прежний подъём/RL. B (L1+L2+B) сохраняет emergency0/3, без HOME wait; electrical disable руки отдельно не валидирован.

Для REMOTE_TEST завершить первый launch после подтверждённого SYSTEM_HOLD/output OFF; повторить команду ног с `operation_profile:=remote_test` и тем же trial=true. Профиль не меняется во время сессии. Нейтраль стиков — zero velocity; затем малые vx/vy/yaw, bounds±0,20/0,10/0,10. Повторить X/A несколько раз, записывая software status и физическое поведение раздельно. NAV/FULL и Sport-mode IMU calibration не смешиваются с этим тестом.

## Файлы и Git

В `src/unitree_ros2_to_real/`: добавлен MakePassiveLowCmd в safety_io header/source, внутренняя phase и bounded publication acknowledgements в r3_commissioning/system_state, bridge/status в go2_r3_commissioning, новый system_passive_test, обновлённые fixtures/controlled-stop/restart/emergency/lifecycle tests, CMake и read-only smoke. README/общий журнал решений/specification/этот отчёт синхронизированы в docs и workspace.

B.2 сохраняется отдельным commit `feat: command bounded passive transition after controlled lie-down`; B.1 сохранён в e2990c6. Push не выполнялся. Код/reference, planner, SDK calibration и RARS owner не менялись.


## Повторная сверка перед commit — 06.10.2026

Повторно прочитаны пути A/X/B, stop acknowledgement, publisher/lease reacquisition, immutable profiles и async generation guards. Найден и исправлен service обход: при output-OFF SYSTEM_HOLD compatibility phase HOLDING позволяла maintenance stand/RL/lie-down менять FSM без включённого output. Все три запроса теперь требуют active custom output; обычный A остаётся единственным корректным restart через readiness/ownership/capture. REQUEST_HOLD/ENABLE_OUTPUT также проверены отрицательной регрессией: output-OFF состояние сохраняется, RL не запускается. Начальный A и действующие output-ON сервисы не изменены.

Согласованная пересборка PASS (`fsm_stepb2_commit_review_build.log`), targeted17/17 PASS (`fsm_stepb2_commit_review_test.log`), финальная полная регрессия30/30 PASS (`fsm_b2_commit_review_regression.log`). Initial A byte-identical; CPU JIT max0,914655 мс, полный agent max1,7335 мс/20 мс. Новых физических команд не отправлялось. Динамика нового X/PASSIVE/A всё ещё требует операторского physical trial.

Команда push:

```bash
git -C /home/ruben/go2_diploma/sim2real/repos/workhop_rl push origin ros2_go2_rars01_real
```
