# Временный лог стиков, RL targets и задержек + настройка kp/kd

07.10.2026. Добавлено диагностическое логирование; команда движения, policy, gains, stand/lie trajectories и watchdogs не изменялись. Физические команды агент не отправлял. Работающий arm owner оставлен нетронутым.

## Что записывается

Отдельный CSV, без печати каждого sample в терминал и без форматирования CSV на IO thread:

- Стики `sticks0=ly`, `sticks1=-rx`, `sticks2=-lx`: до deadband/масштабирования. Исходные rx/lx восстанавливаются обратным знаком.
- `requested0..2`: выбранное отображение стиков после deadband0,01 и limits±0,5. `command0..2`: фактическая команда, переданная policy (vx/ vy / wz).
- Policy row: clipped action12 до action_scale, q_des12 после scale/default/position clamp в hardware order, accepted/rejected result, compute_ms, полный job_age_s и возраст захваченных LowState/remote/arm/target.
- IO row: реально опубликованные q_des/kp/kd для12 ног, measured q/dq, IMU quaternion/gyro, число sent packets, возраст последнего принятого policy-result. IO строки записываются примерно50Гц, но min/max интервалов собираются по **каждому успешному publish500Гц**, чтобы увидеть микрозадержки между строками.
- Policy interval показывает расстояние между стартами диагностируемых inference jobs; job_age включает подготовку/вычисление/ожидание acceptance, compute_ms — только Act и штатное извлечение12 targets.

`kind=0` — IO row, `kind=1` — policy row. `phase` — числовой SystemPhase из `include/system_state.hpp`. `steady_ns` — monotonic timestamp IO publish или policy completion, не календарное время. Для policy начало job можно восстановить как `steady_ns − job_age_s×1e9`; возраст источников относится к **захваченному** snapshot, не подменяется свежими сообщениями во время inference.

Все массивы ног/targets/action в CSV имеют hardware порядок FR,FL,RR,RL (hip/thigh/calf). Quaternion — xyzw, gyro — xyz. `action` значим только в policy rows, `kp/kd` — только в IO rows. Для rejected policy target — кандидат, он не отправляется роботу. Для принятого результата публикация может произойти на следующем IO tick.

В PASSIVE q_des содержит stop sentinel2.146E9, а не угол для движения: такие строки не использовать для расчёта Δtarget ходьбы. Большой publish/job interval через штатное output OFF или PD этап учитывает намеренную паузу; анализировать интервалы вместе с phase.

Это диагностические samples, не полный315-D tensor dump и не запись всех500Гц packets. Min/max interval помогает искать publish задержку, но50Гц measured samples не доказывают отсутствие быстрых колебаний моторов. Переходы и fault reason по-прежнему записываются в обычный ROS log.

## Как включить на одну проверку

В новом leg launch добавьте три аргумента к существующей команде:

```bash
motion_diagnostics_enabled:=true \
 motion_diagnostics_duration_s:=120.0 \
 motion_diagnostics_path:=/home/ruben/go2_diploma/sim2real/runtime/go2_motion_trace
```

Полная команда, которую запускает оператор:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /home/ruben/go2_diploma/sim2real/setup.bash
source /home/ruben/go2_diploma/sim2real/install_fsm/local_setup.bash

ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  operation_profile:=remote_test \
  controlled_stop_lie_down_trial:=true \
  config_path:=/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt \
  network_interface:=enP8p1s0 \
  motion_diagnostics_enabled:=true \
  motion_diagnostics_duration_s:=120.0 \
  motion_diagnostics_path:=/home/ruben/go2_diploma/sim2real/runtime/go2_motion_trace
```

В начале launch выводится точный путь `go2_motion_trace-<steady-id>.csv`. По умолчанию запись выключена. Окно120с начинается с первого принятого диагностического sample, обычно первого собственного LowCmd publish после A: ожидание оператора перед A не расходует окно. Допустимая длительность больше0 и до600с.

### Остановить запись раньше

В другом терминале с тем же ROS окружением:

```bash
ros2 service call /go2/diagnostics/stop_motion_log std_srvs/srv/Trigger '{}'
```

Это прекращает приём новых строк; остаток ограниченной очереди дописывается отдельным writer thread. Управление ногами, Sport, состояние руки и текущая policy команда от этого service не меняются. Автоматическая остановка выполняется по истечении duration и без новой команды оператора. В `/go2/locomotion_status` поля `motion_diagnostics_active` и `motion_diagnostics_dropped` показывают активность и пропуски очереди. Startup parameter enabled остаётся исходной настройкой, поэтому проверять завершение следует по active, а не по `ros2 param get`. Для обычного следующего запуска уберите аргументы диагностики или задайте `motion_diagnostics_enabled:=false`. Включение/длительность/path задаются только при старте; stop service не запускает вторую сессию записи.

## Как ограничена нагрузка

При выключенной диагностике logger не создаётся: нет writer thread, CSV, извлечения action для логов или сбора publish intervals. В control paths остаются короткие проверки наличия/активности logger. После auto-stop/stop service не принимаются и не формируются новые samples; writer дописывает очередь и завершает поток.

При включении: фиксированная очередь256 samples, `try_lock` на producer — не ждать ни диска, ни владельца очереди. При переполнении/конкуренции sample пропускается и увеличивается drop counter. CSV formatting/file writes только в отдельном потоке. Диагностика имеет ненулевую нагрузку при включении; данные предназначены для короткого теста. Нет sleep/flush/open/синхронной печати tensor в IO500Гц. Обычные существующие ROS status/transition logs этим флагом не отключаются.

## Что искать в записи

1. Стик резко меняется → requested/command резко меняются → на том же/следующем policy frame растёт Δaction/Δtarget. Это аргумент в пользу command acceleration limiting; уменьшение kp не удаляет сам скачок target.
2. Стик около нуля, requested прыгает0↔ненулевое у deadband0,01: проверить шум/нейтраль и необходимость hysteresis. RL_ZERO↔RL_ACTIVE сам по себе не означает history reset.
3. Command и target плавные, measured q/dq/IMU колеблются: искать tracking/контакт/нагрузку/моторную динамику, а не менять input mapping без основания.
4. Растут interval_s, io_max_s, job_age_s или captured data ages: отличить задержку получения данных, scheduling, compute и публикации. compute_ms может быть нормальным при большом полном job_age.
5. На X проверить последнюю принятую policy строку, rejected pre-X ticket, frozen IO target и новое accepted zero-command result; затем плавную смену q_des в lie-down. Числовая phase сопоставляется с обычным R3 transition log.

Обновление после физического теста 07.10.2026: оператор уже выбрал RL20/1,1; startup последнего запуска это подтверждает. Для следующей диагностической записи сохранить эти gains. [Анализ последнего лога](CODEX_LATEST_PHYSICAL_LOG_ANALYSIS.md).

## Где менять RL и подъём

**При команде запуска выше реально читается файл:**

`/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml`

Ключи в секции `go2_rars01`:

| Режим | kp key | kd key | Сейчас |
| --- | --- | --- | --- |
| RL, включая zero-command RL до HOME по X | `rl_kp` | `rl_kd` | 20 / 1,1 в текущем operator profile |
| Подъём, fixed capture, stand hold, measured PD и штатный lie-down | `fixed_kp` | `fixed_kd` | 40 / 1 |

Каждый ключ содержит **12 значений**. Изменить все12 одинаково для общего теста. `fixed_*` используется совместно несколькими этапами: изменение «для подъёма» затронет также fixed hold и штатное укладывание.

Пример отдельного RL теста20/1,1, без изменения fixed gains:

```yaml
go2_rars01:
  rl_kp: [20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20]
  rl_kd: [1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1]
  fixed_kp: [40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40]
  fixed_kd: [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
```

Это **фрагмент**, не полный файл: не заменять им остальные actor/deployment ключи. Gains читаются при старте процесса; правка YAML не меняет уже запущенный controller. Для reload нужен новый leg launch. Стартовый log печатает реально загруженные fixed_kp/fixed_kd/rl_kp/rl_kd; CSV IO rows показывают выбранные значения при publish.

Версионные файлы для сохранения настройки:

- `sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml` — единственный редактируемый source operator config; копии в runtime/ больше нет.
- `sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml` — production/default config. Он действует, когда выбран именно этот config_path (или package default), а не explicit commissioning config_path.

Не редактировать generated install/build configs для постоянной настройки. Array order в YAML — actor FL,FR,RL,RR; loader переставляет leg gains в hardware order. При равных12 значениях перестановка результата не меняет. B/emergency kd задан отдельно в `real_deployment.r3_commissioning.emergency.motor_kd`; это не RL/fixed kd.

## Проверки и сохранение

- Release/Jazzy/aarch64 candidate build install_fsm — PASS; итоговый node собран с диагностикой.
- Logger unit test — PASS: CSV103 columns и числовой roundtrip command/target; bounded queue; concurrent overload/drop; auto-stop; explicit stop/drain; invalid duration.
- Функциональный CTest `-E '^policy_cpu_test$'` — **33/33 PASS**. Отдельный CPU timing benchmark в этой задаче намеренно не повторялся; прежний FAIL (полный Agent max24,9мс при20мс) остаётся незакрытым. Общий timing PASS не заявляется.
- ROS read-only localhost/domain223 smoke — PASS: enabled status, stop service, active=false после остановки, controller продолжает работать, zero /lowcmd publishers. Физических SDK/serial команд нет.
- verify_r1/r2/r3 и git diff --check — PASS.
- Первая сборка выявила несовместимость generic service lambda с Jazzy rclcpp; callback заменён на явные Request/Response types. Первый regression выявил устаревшую launch fixture и строковый assertion публикационного ACK; fixtures обновлены, сохранена проверка publish→timestamp→ACK→diagnostics, итоговый прогон PASS.

Логи: runtime/motion_trace_verified_build.log, motion_trace_test_build.log, motion_trace_final_regression.log, motion_trace_ros_smoke.log, motion_trace_ros_node.log, motion_trace_verify.log. Промежуточные отказы сохранены в motion_trace_build.log и motion_trace_regression.log.

Новые файлы: include/motion_trace.hpp, src/motion_trace.cpp, tests/motion_trace_test.cpp. Изменены node, launch, CMake, launch/lifecycle tests и документация. CSV smoke file содержит только header, потому что read-only smoke не посылает actuator output; реальные числовые поля проверены synthetic logger unit test. Сборка и отключение диагностики готовы; физическая запись требует запуска оператором.

Изменения пока не закоммичены: последняя просьба была добавить логирование и отчёт; новый commit не запрашивался. Предыдущий commit92b9431 сохранён. В последнем физическом запуске подробная CSV диагностика не включалась; анализ transition log сохранён отдельно.
