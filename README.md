# Go2 + RARS01: запуск Sim2Real

Jetson Orin Nano · ROS 2 Jazzy · ветка `ros2_go2_rars01_real`. Это единственная актуальная инструкция запуска нашей Sim2Real реализации. Отчёты и решения находятся в `docs/sim2real/`; upstream README отдельных SDK/симулятора сохраняются как документация зависимостей.

## 1. Окружение в каждом терминале

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /home/ruben/go2_diploma/sim2real/setup.bash
source /home/ruben/go2_diploma/sim2real/install_fsm/local_setup.bash
```

`install_r1` используется как underlay, `install_fsm` содержит текущие executables. Read-only Sim2Sim reference и его Humble build/install не используются как runtime. Сохранить настройки ROS domain/RMW/CycloneDDS физической сети; Ethernet интерфейс — `enP8p1s0`.

## 2. Рука: один HOME owner

GUI SDK, второй owner и systemd owner одновременно не запускать. Если healthy HOME owner уже работает, повторно запускать его не нужно.

```bash
ros2 run unitree_legged_real rars_r3_owner --ros-args \
  -p read_only:=false -p connect_serial:=true \
  -p sdk_config_path:=/home/ruben/go2_diploma/sim2real/repos/rars01_graspnet/config/default.yaml \
  -p config_path:=/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml \
  -p device_path:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3172366B3233-if00 \
  -p lock_directory:=/home/ruben/go2_diploma/sim2real/runtime/rars_serial_leases \
  -p journal_path:=/home/ruben/go2_diploma/sim2real/runtime/rars_auto_home/enable-journal
```

После connect: задержка10с → один enable → семь нулевых HOME targets100Гц. Feedback появляется после enable. Journal диагностический; runtime fault прекращает сессию без auto re-enable. Return-home RPC не включает выключенные моторы. Завершение leg controller само по себе не завершает arm owner.

В `src/unitree_ros2_to_real/deployment/` подготовлены `rars01-owner.service`, env example и launcher для отдельной установки; `Restart=no`. Ниже используется ручной owner. `workspace_scripts/` — versioned snapshot помощников workspace; относительные пути рассчитаны на копирование в `sim2real/scripts/`.

## 3. Ноги: управление RL с пульта

В отдельном терминале после загрузки окружения:

```bash
ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  operation_profile:=remote_test \
  controlled_stop_lie_down_trial:=true \
  config_path:=/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt \
  network_interface:=enP8p1s0
```

Launch ждёт A. До takeover нужны fresh LowState/remote/Sport, arm HOME и остальные gates. Trial-флаг разрешает текущую операторскую проверку lie-down, не меняет production validation. Текущий физический тест выявил скачки policy targets и автоматический fault; их причины ещё не исправлены: [анализ CSV](docs/sim2real/CODEX_CSV_AUTO_SHUTDOWN_ANALYSIS.md).

Для RL с постоянной zero velocity заменить профиль на `operation_profile:=rl_zero_test`. Zero velocity — нулевая команда движения, а не нулевые actions суставов.

## 4. Пульт и цикл

Комбинацию удерживать ≥0,75с, затем отпустить. Приоритет B > X > A.

| Управление | Действие |
| --- | --- |
| L1+L2+A | Автоматически release Sport при необходимости → measured capture → stand6с → fixed hold4с → reset policy → RL |
| L1+L2+X | Закрыть команды скорости → zero RL до HOME → measured PD capture → плавный lie-down8с → reached/settle →10 PASSIVE packets → output OFF / SYSTEM_HOLD |
| L1+L2+B | Emergency damping ног kp=0/kd=3 при действующей eligibility; без lie trajectory и auto recovery |
| `ly` | Вперёд/назад ±0,5м/с |
| `-rx` | Вбок ±0,5м/с |
| `-lx` | Поворот ±0,5рад/с |

Deadband0,01; command ramp сейчас отсутствует. Нейтраль оставляет RL с zero velocity. После X Sport сам не включается. Новый A из SYSTEM_HOLD снова проверяет ownership и захватывает текущую измеренную позу. Ctrl+C завершает процесс и не заменяет B.

Approved lie target в hardware FR/FL/RR/RL порядке:

```text
[0.01,1.30,-2.70, -0.01,1.30,-2.70, -0.30,1.30,-2.70, 0.30,1.30,-2.70]
```

Lie tolerance0,15рад, settle0,2с. Без trial при production validation=false X после HOME сохраняет zero RL и показывает blocker. HOME timeout сохраняет zero RL; lie timeout сохраняет последний planned fixed target. PASSIVE — mode0, stop sentinels, kp/kd/tau0; локальная публикация не подтверждает электрическое отключение моторов. Arm emergency disable закрыт до отдельной validation.

## 5. Статус и CSV диагностика

```bash
ros2 run unitree_legged_real r3_status.py --timeout 3600
```

Однократно: `ros2 run unitree_legged_real r3_status.py --once`. Полный raw payload:

```bash
ros2 topic echo /go2/locomotion_status --full-length
ros2 run unitree_legged_real go2_mode_switch --interface enP8p1s0 --status
```

После успешного X: custom_leg_output=OFF, output_enabled=false, publisher/lease отсутствуют, passive_packets_sent=10, sent_packets не растёт. Watchdogs accepted result и in-flight job —40мс, arm captured inputs — отдельно0,25с. При fault сохранить reason и policy ages.

Для записи CSV добавить к leg launch:

```bash
motion_diagnostics_enabled:=true \
  motion_diagnostics_duration_s:=240.0 \
  motion_diagnostics_path:=/home/ruben/go2_diploma/sim2real/runtime/go2_motion_trace
```

В startup появится `Temporary motion diagnostics:` с точным CSV path. Окно240с начинается с первого sample после A и автоматически заканчивается; следующий обычный launch без flags CSV не пишет. По умолчанию диагностика выключена. Статус: motion_diagnostics_active/dropped. Досрочно остановить только запись:

```bash
ros2 service call /go2/diagnostics/stop_motion_log std_srvs/srv/Trigger '{}'
```

[Описание полей и нагрузки](docs/sim2real/CODEX_TEMP_MOTION_LOGGING_AND_GAINS.md). Последний120-секундный CSV закончился за10,5с до fault;240с предложены для записи конца следующей сессии.

## 6. Где менять параметры

Единственный source operator YAML: `src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml`. Передаётся явно через config_path; читается при новом запуске.

| Этап | Ключи секции go2_rars01 | Текущий operator profile |
| --- | --- | --- |
| RL | rl_kp / rl_kd | 20 / 1,1, по12 значений |
| Stand, fixed hold, PD capture, lie-down | fixed_kp / fixed_kd | 40 / 1, по12 значений |

Production/default — `config/go2_rars01_real.yaml`, RL25/1. B gains заданы отдельно в `real_deployment.r3_commissioning.emergency.motor_kd`. Изменения source не обновляют работающий процесс. Generated install/build YAML не редактировать; runtime содержит logs/CSV/leases/journals.

## 7. Read-only и автономная навигация

Без физических команд:

```bash
ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  operation_profile:=read_only \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt
```

Другие immutable profiles: arm_test, leg_safety_test, rl_zero_test, remote_test, nav_test, full_mission. NAV/FULL пока блокируются без readiness/cancellation adapters: одного `/cmd_vel` недостаточно. NAV использует TwistStamped `/cmd_vel`, freshness≤0,25с, limits±0,20/0,10/0,10; `pathFollower.sendSportCommand=false`. REMOTE не слушает NAV.

IMU calibration `autonomy_nav_go2:ros2_Jazzy` — отдельный Sport-mode тест с командами движения: custom controller остановлен/LowCmd OFF, штатный Sport вручную подтверждён. SYSTEM_HOLD после X оставляет Sport RELEASED и этому условию не соответствует. Команда в собранном NAV workspace: `ros2 run calibrate_imu calibrate_imu`; результат `~/Desktop/imu_calib_data.yaml`.

## Документация и состояние проверки

- [Архитектура папок](docs/sim2real/PROJECT_STRUCTURE.md).
- [Общий журнал решений](docs/sim2real/CODEX_SIM2REAL_DECISIONS.md).
- [Observation/action contract и движение](docs/sim2real/CODEX_RL_DATA_AND_MOTION_AUDIT.md).
- [Последний fault и реальные скачки targets](docs/sim2real/CODEX_CSV_AUTO_SHUTDOWN_ANALYSIS.md).

Functional regression33/33 PASS, build/read-only diagnostics smoke PASS. Прежний offline CPU timing FAIL остаётся незакрытым; физический последний fault — policy_result_stale при41,858мс, затем outputOFF из-за Sport-status age>0,5с. Успешные A/X циклы предыдущего теста не означают устранение резкости actor.
