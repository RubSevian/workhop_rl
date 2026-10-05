# Go2 + RARS01: запуск Sim2Real

Jetson Orin Nano · ROS 2 Jazzy · workspace `/home/ruben/go2_diploma/sim2real`.

Актуальные параметры и результаты проверок: [CODEX_SIM2REAL_DECISIONS.md](CODEX_SIM2REAL_DECISIONS.md).

## 1. Перед запуском

- Завершите прежний launch ног. Новая сборка не обновляет уже запущенный процесс.
- Подключите Go2 к `enP8p1s0`, STM32 руки — к USB.
- Для первого теста навигацию не запускайте.
- Для STM32 должен работать один owner: GUI SDK и другие serial-клиенты закройте. Уже работающий owner в `HOLD_HOME` оставьте; второй не запускайте.

## 2. Окружение — в каждом терминале

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /home/ruben/go2_diploma/sim2real/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP8p1s0"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
```

Если `source` сообщил об ошибке, не переходите к запуску. Humble и окружение `sim2sim_reference` для этого workspace не используются.

## 3. Терминал 1: рука

Пропустите этот шаг, если owner уже работает и держит `HOLD_HOME`.

```bash
ros2 run unitree_legged_real rars_r3_owner --ros-args \
  -p read_only:=false \
  -p connect_serial:=true \
  -p sdk_config_path:=/home/ruben/go2_diploma/sim2real/repos/rars01_graspnet/config/default.yaml \
  -p config_path:=/home/ruben/go2_diploma/sim2real/repos/workhop_rl/src/unitree_ros2_to_real/config/go2_rars01_real.yaml \
  -p device_path:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3172366B3233-if00 \
  -p lock_directory:=/home/ruben/go2_diploma/sim2real/runtime/rars_serial_leases \
  -p journal_path:=/home/ruben/go2_diploma/sim2real/runtime/rars_auto_home/enable-journal
```

После соединения owner ждёт 10 секунд, один раз включает моторы и удерживает семь нулевых HOME targets с частотой 100 Гц. Дождитесь `HOLD_HOME`, оставьте терминал работающим. Feedback STM32 появляется после enable.

Journal используется только для диагностики и не блокирует новый запуск. При runtime fault текущая сессия останавливается: после проверки причины завершите owner и запустите его заново явно. Файлы serial leases не удаляйте. Подготовленная systemd-служба не запускается этим README.

## 4. Терминал 2: первый тест RL с нулевой скоростью

```bash
ros2 launch unitree_legged_real go2_rars01_r3_commissioning.launch.py \
  config_path:=/home/ruben/go2_diploma/sim2real/runtime/r3_first_rl_zero.yaml \
  model_path:=/home/ruben/go2_diploma/sim2real/weights/policy_2.pt \
  network_interface:=enP8p1s0 \
  control_mode:=remote_test \
  motion_commands_enabled:=false \
  remote_auto_sequence:=true \
  read_only:=false
```

Launch сам не поднимает робота. В этом тесте policy получает нулевую скоростную команду, стики игнорируются, A/X/B действуют. Policy продолжает выдавать суставные цели — это не нулевые action.

Используется экспериментальный профиль, ранее принятый оператором. Его разрешающие флаги не означают, что физический тест уже пройден.

## 5. Терминал 3: статус и начало теста

```bash
ros2 topic echo /go2/locomotion_status
```

До A дождитесь:

- `arm_ready: true`, `arm_home_ready: true`;
- свежих LowState/remote/Sport и отсутствия `fault_latched`;
- `STOCK` + `SPORT_ACTIVE`, либо `DISARMED` + `SPORT_RELEASED`, если Sport уже отключён.

Затем удерживайте **L1+L2+A не менее 0,75 с** и отпустите. Последовательность:

```text
Проверки → отключение Sport → ожидание освобождения LowCmd
→ измеренный hold → линейный подъём 6 с → удержание 4 с → RL_ZERO
```

В нулевом тесте ожидаются `state: RL_ZERO`, `motion_command: [0, 0, 0]`. Ожидание SDK/discovery добавляется к времени подъёма.

## 6. Пульт — одинаковый в обоих режимах

Все комбинации удерживаются **не менее 0,75 с**; перед повтором отпустите кнопки.

| Комбинация | Действие |
|---|---|
| **L1+L2+A** | Полный цикл до RL; после X — новый явный цикл подъёма и RL |
| **L1+L2+X** | Выход из RL, удержание измеренной позы с Kp=40, Kd=1 |
| **L1+L2+B** | Emergency: пассивный damping Kp=0, Kd=3; после него RL заблокирован |

B не задаёт траекторию в лежачую позу. X/B не выключают руку. **Ctrl+C завершает процесс и не заменяет damping по B.**

При fault сохраните `state`, `fault`, `fault_latched`, `policy_ms` и вывод переходов. Не запускайте автоматический повтор опыта.

## 7. Тест управления стиками

В команде раздела 4 замените только:

```text
motion_commands_enabled:=true
```

| Ось workshop | Команда | Предел |
|---|---|---|
| `ly` | Вперёд / назад | ±0,20 м/с |
| `-rx` | Влево / вправо | ±0,10 м/с |
| `-lx` | Поворот | ±0,10 рад/с |

Нейтральные стики дают нулевую скорость; RL остаётся включённым. `/cmd_vel` в этом режиме игнорируется. Не меняйте режим перезапуском controller, оставив робот без выбранного удержания/stock.

## 8. Режим навигации

В команде раздела 4 используйте:

```text
control_mode:=autonomy
motion_commands_enabled:=true
```

- Скорость приходит только из `/cmd_vel` типа `geometry_msgs/msg/TwistStamped`, с теми же пределами.
- В `pathFollower` обязательно `sendSportCommand:=false` — стек передаёт запросы RL, не Sport RPC.
- Timestamp должен быть свежим: максимум 0,25 с, без времени из будущего. Без свежих команд скорость становится нулевой.
- Стики не управляют скоростью; A/X/B продолжают действовать. Навигация не включает takeover сама.

Эта команда запускает controller. LiDAR, IMU-калибровка и полный навигационный стек подключаются отдельным этапом.

## 9. Диагностика

| Симптом | Что проверить |
|---|---|
| `package not found` | Окружение раздела 2 загружено без ошибок |
| Нет commissioning status | Launch controller работает; во всех терминалах одинаковые domain/RMW/interface |
| `FAULT_LATCHED` у руки | Посмотреть текущий `error`; watchdog останавливает сессию. После устранения причины требуется новый явный запуск owner |
| A не запускает цикл | `remote_event`, `remote_sequence_message`, `remote_sequence_blockers`, `arm_error`, готовность HOME |
| Не включается RL | `state`, `fault`, `policy_ms` и сохранённые строки `R3 transition` |

Для policy отдельно выводятся `policy_result_age_s` (время с принятия ответа), `policy_inference_age_s` (возраст текущего расчёта) и `policy_observation_age_s` (возраст LowState последнего принятого расчёта). Watchdog ответа и незавершённого расчёта — 40 мс; данные руки проверяются отдельно. В момент fault эти значения сохраняются в transition log.

Для проверки Sport без переключения режима:

```bash
ros2 run unitree_legged_real go2_mode_switch --interface enP8p1s0 --status
```


Для чтения последней диагностической записи руки:

```bash
cat /home/ruben/go2_diploma/sim2real/runtime/rars_auto_home/enable-journal
```

Содержимое journal не проверяется при старте. Старые FAULT, ENABLE_ATTEMPT и повреждённые записи не мешают запуску; архивирование для перезапуска больше не требуется. В текущем процессе fault остаётся защёлкнутым и автоматического re-enable нет.
