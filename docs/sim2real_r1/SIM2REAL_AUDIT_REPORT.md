# Перенос дипломного Sim2Sim на Unitree Go2

**Аудит и план переноса · 3 октября 2026 · Jetson Orin Nano / ROS 2 Jazzy**

## Итог

**Основу переносить можно, но текущий `ros2_go2_rars01_real` ещё не готов к реальному запуску.** Ветка точно совпадает с `ros2_go2_rars01_sim`; реальный контроллер внутри неё использует старый конфиг, не получает состояния руки и не подключён к навигационному `/cmd_vel`. Есть ошибки порядка суставов при stand-up/hold и отсутствуют необходимые блокировки публикации команд.

Новая `sim2real/weights/policy_2.pt` действительно является TorchScript actor **315 → 12**. SHA-256 совпадает с manifest из reference; загрузка и CPU forward прошли. Это подтверждает бинарную совместимость по размерности, но не доказывает соответствие физики обучения реальному роботу: training commit/checkpoint в manifest не заполнены.

Правильная стратегия: сохранить RL-математику из Sim2Sim; выборочно перенести real/Jazzy/safety решения из `go2_nav`; добавить настоящий feedback/target bridge RARS01; отдельно восстановить нужные дипломные изменения навигации поверх `autonomy_nav_go2:ros2_Jazzy`.

**В ходе аудита:** исходники и reference не изменялись; сборка, установка зависимостей, ROS launch, публикация LowCmd, переключение Sport Mode, подключение serial и команды движения не выполнялись. Единственный созданный артефакт — этот отчёт. Python-проверка использовала установленный Torch из внешнего venv, без импорта контроллеров оборудования и с отключённой записью bytecode.

## 1. Проверенная исходная точка

Все четыре рабочих Git-дерева на момент проверки чистые. Проверялись локальные checkout и локальные remote-tracking refs; `fetch` не выполнялся.

| Компонент | Рабочая ветка / SHA | Результат |
|---|---|---|
| `workhop_rl` | `ros2_go2_rars01_real`, `b5085333029f8174d11884fcd457ddbde04ad6c8` | Совпадает с локальной `ros2_go2_rars01_sim` и reference |
| Источник старых real-решений | `origin/go2_nav`, `e20e8ad659dd2a1ec58ffacf536eeb856f4ba254` | Читать через `git show`; не сливать целиком |
| `autonomy_nav_go2` | `ros2_Jazzy`, `a5f3e67f8116394fbbd2e3364ca9e8e9fa2cd2c9` | Отличается от дипломной navigation-ветки |
| `rars01_graspnet` | `jetson`, `8efeb6ffaa042617f07e32c5f2dff939d8241b73` | Совпадает с reference |
| `rars01_description` | `go2_arm`, `2df2bf4f9a28437124b4eedf6e1777c2f497b709` | Совпадает с reference |
| Navigation reference | `8427473abd8ad4fc40c5d26b5af5d78b67524305` | Содержит более поздние дипломные исправления |

### Окружение, доступное аудиту

| Проверка | Фактически обнаружено |
|---|---|
| Архитектура | `aarch64` |
| ОС | Ubuntu `24.04.5`, Noble |
| ROS | `/opt/ros/jazzy` существует; текущая оболочка ROS не source-ила |
| JetPack | пакет `nvidia-jetpack 7.2.1-b49` |
| L4T | `/etc/nv_tegra_release`: `R39`, revision `2.1` |
| CUDA compiler | `nvcc 13.2.86` |
| ROS CycloneDDS | `ros-jazzy-rmw-cyclonedds-cpp 2.2.4-1noble.20260902.140937` |
| Torch | `2.13.0+cu132`, CUDA build `13.2`, C++11 ABI включён |
| Старый LibTorch | `/home/ruben/libtorch` отсутствует |
| GPU из сессии | `torch.cuda.is_available() == False`; ошибки NvRmMemInitNvmap, `/dev/nvmap` недоступен |

Ubuntu Noble ARM64 соответствует целевой платформе Jazzy. Это подтверждается [официальным сообщением Open Robotics](https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released).

**Ограничение GPU-проверки:** CUDA-компилятор и CUDA-сборка Torch присутствуют, но GPU forward не выполнен. По ограниченной сессии нельзя определить, вызвано это отсутствием проброса устройств или проблемой драйвера на хосте. Работоспособность CUDA runtime остаётся непроверенной.

## 2. Что сохранить из `ros2_go2_rars01_sim`

| Часть | Решение |
|---|---|
| `unified_observation_contract.hpp/.cpp` | Сохранить порядок полей, масштабы, projected gravity, историю и reset без изменения математики |
| Unified-ветка `rl_agent.cpp` | Сохранить вход 315, выход 12, previous action, action scale и default pose |
| `go2_rars01_unified.yaml` | Сохранить контракт; сделать отдельный real-профиль с той же семантикой |
| `navigation_command_adapter.hpp/.cpp` | Сохранить базовую семантику finite-check, steady-clock watchdog, navigation gate и физические единицы; real-лимиты сделать конфигурируемыми |
| `tests/reference_unified_observation.py`, contract/reset/config tests | Сохранить как проверку совпадения с Sim2Sim |
| `rars01_description` URDF и геометрия | Сохранить; аппаратные измерения и калибровки не заменять симуляционными значениями |
| GraspNet IK, limit contract, FK, trajectory math | Сохранить существующую проверенную математику; менять транспорт и интеграцию |
| MuJoCo-модели, генераторы, виртуальный payload, shadow checker | Сохранить как offline regression reference; исключить из real runtime/build по умолчанию |

«Без изменений» относится к поведению RL-контракта, а не к старому CMake и runtime wiring. Сборочные файлы `unitree_rl_controller` придётся адаптировать под установленный Torch.

Симуляционный `mujoco_sim.cpp` полезен как источник arm-target и navigation wiring, но не как real runtime: он читает руку из `motor_state[12..17]`, использует physics-scheduled inference и обслуживает GLFW. Эти предположения на настоящем Go2 неприменимы.

## 3. Что взять из `go2_nav`

| Источник в `origin/go2_nav` | Что переносить |
|---|---|
| `jazzy_setup.sh` | Jazzy underlay, внешний `Torch_DIR`, переменный сетевой интерфейс, рабочие overlays |
| `src/unitree_rl_controller-ros2/CMakeLists.txt` | Подход к native Torch и C++-стандарту; сохранить unified sources/tests текущей ветки |
| `src/unitree_ros2_to_real/CMakeLists.txt` | SDK2 imported target, aarch64, отключаемый legacy sim, отдельный mode-switch, отсутствие SDK2 linkage у ROS-контроллера |
| `src/unitree_ros2_to_real/src/ros2_rl_go2.cpp` и соответствующий header | `TwistStamped`, параметры путей/лимитов, watchdog LowState/cmd_vel, model-loaded gate, fault latch, timing diagnostics |
| Те же controller-файлы | Правильный mapping в stand-up/hold, непрерывная команда на переходных тиках, плавный переход в hold |
| `src/go2_mode_switch.cpp`, `src/go2_motion_mode.cpp`, `include/go2_motion_mode.hpp` внутри пакета | Отдельный SDK2-процесс для статуса/управления Sport Mode; переносить исходники, не запускать в аудите |

**Адаптировать, а не копировать буквально:**

- Старый `Agent::ResetHistory()` заменить на текущий `ResetPolicyState()`, вызываемый на входе в RL после обновления реальных observations.
- Старое имя `go1` и legacy weights не переносить: real-профиль должен явно выбирать unified config и `sim2real/weights/policy_2.pt`.
- Не переносить автоматический подъём как поведение по умолчанию. Startup — disarmed, actuator output выключен.
- `low_level_mode_verified` в старом решении — переданный пользователем флаг, а не независимое доказательство текущего состояния сервиса. Нужен явный процесс preflight и контроль единственного владельца привода.
- Старый fault latch просто прекращает публикацию. Это не доказанный физический emergency stop: поведение при исчезновении LowCmd надо определить для фактической прошивки Go2.
- Числа таймаутов/скоростей из `go2_nav` — исходные настройки для проверки, а не доказанные границы новой policy.
- Hardcoded внешний venv из `jazzy_setup.sh` заменить параметром окружения; не создавать скрытую зависимость от другого рабочего проекта.

## 4. RL observation/action contract

### Подтверждено по исходникам и бинарнику

Файл: `sim2real/weights/policy_2.pt`.

SHA-256: `a70b18b88e6ee2fe32de4904559404f4f672dff2396e95ca2c851fdefc5a8cdc`.

SHA совпадает с `sim2sim_reference/go2_diploma_sim2sim/weights/POLICY_MANIFEST.yaml`. Внутри архива имя `policy_1` — имя TorchScript archive root, оно не меняет идентичность файла.

Actor: `315 → 512 → 256 → 128 → 12`, ELU между скрытыми слоями. CPU загрузка через установленный Torch и forward на нулевом `[1,315]` дали `[1,12]`, все значения конечные. Проверка размерности не является проверкой устойчивости походки.

| Срез одного кадра, Python half-open | Содержание | Преобразование |
|---|---|---|
| `[0:3]` | Base angular velocity | × `0.25` |
| `[3:6]` | Projected gravity | Inverse rotation `[0,0,-1]` нормализованным quaternion |
| `[6:9]` | `vx, vy, wz` | × `[2,2,0.25]`, до scaling — м/с, м/с, рад/с |
| `[9:21]` | 12 leg positions | `(q − q_default) × 1` |
| `[21:33]` | 12 leg velocities | × `0.05` |
| `[33:45]` | Previous actor action | Предыдущее clipped action, до action scale |
| `[45:51]` | 6 arm positions | Абсолютные суставные координаты, рад |
| `[51:57]` | 6 arm velocities | × `0.05` |
| `[57:63]` | 6 arm targets | Суставные target-позиции, рад |

История: пять кадров, **от старого к новому**, `[1,315]`; reset повторяет первый актуальный кадр пять раз и обнуляет previous action. Policy rate — **50 Hz**. `decimation=4` описывает training/sim contract; на железе это не дополнительное деление таймера 50 Hz на четыре.

Actions: 12 ног; `q_target = q_default + 0.25 × action`. Observation/action clip — `±100`; в `Agent::Act()` дополнительно используется общий position clamp `±3.5`. Это не индивидуальные аппаратные пределы суставов и не torque safety.

Default pose в policy order:

```text
FL [ 0.1, 0.8, -1.5]    FR [-0.1, 0.8, -1.5]
RL [ 0.1, 0.8, -1.5]    RR [-0.1, 0.8, -1.5]
```

Policy order: **FL, FR, RL, RR**, внутри ноги hip/thigh/calf. Go2 motor order в текущем CRC header: **FR, FL, RR, RL**. Motor→policy mapping: `[3,4,5,0,1,2,9,10,11,6,7,8]`.

Quaternion: Go2 LowState трактуется как `[w,x,y,z]`, контракт получает `[x,y,z,w]`. Проверить ориентацию и оси на неподвижной записи реального IMU, не подменять RL IMU преобразованным LiDAR IMU без доказанного преобразования.

### Блокирующие расхождения текущего real-контроллера

1. `InterfaceRos` выбирает `weights/go2/config.yaml` и legacy model name. Новый unified config и `policy_2.pt` не подключены.
2. `ros2_rl_go2.cpp` не заполняет `arm_pos`, `arm_vel`, `arm_target`: остаются нули из `Agent::InitObservations()`.
3. Реальные arm observations должны поступать из RARS feedback. Нельзя читать руку из запасных Go2 LowState slots или подставлять commanded position вместо измеренной.
4. В stand-up и hold используется индекс `i` вместо `net2joint_indexes[i]`: для асимметричных hip defaults это даёт неверные знаки для сторон робота. Mapping требуется для позиций, gains и limits во всех режимах.
5. History и previous action не сбрасываются на повторном входе в RL; `InitRL()` вызывается только один раз.
6. Нет unified model-loaded gate: исключение загрузки логируется, но node/timer продолжают работать.
7. Нет обработки stale LowState/joystick, нет navigation gate, нет cmd_vel subscriber.
8. `init_cmd()` задаёт stop-сентинелы, но результат затем полностью заменяется новым `LowCmd` из `update()`. Header заполнение закомментировано; оставшиеся motor slots и переходные ветки требуют отдельной проверки. Сам CRC не исправляет содержание команды.
9. `torque_limits` участвуют в `ComputeTorque()`, но real position-command путь задаёт `tau=0`, kp/kd на моторе и не вызывает эту функцию. Нельзя считать, что software torque clipping уже защищает real robot.

### Чего бинарник не доказывает

Training commit/checkpoint/run_name не заполнены. Указанный в PROJECT_MANIFEST training SHA — provenance reference, но связь именно этого export с конкретным checkpoint не зафиксирована. Понадобятся параметры обучения: массы/инерции Go2+руки, arm target semantics, диапазоны скоростей/поз руки, PD/latency, payload и randomization.

Нулевая arm target при отсутствии команды совпадает с текущим симуляционным home. Для real bridge надо передавать target, фактически удерживаемый RARS-контроллером, и подтвердить, что именно такая семантика соответствует training. При потере target/feedback RL не должен продолжать с выдуманными нулями.

## 5. Замена Humble-зависимостей и сборка Jazzy/aarch64

| Сейчас | Требуемое решение |
|---|---|
| `/opt/ros/humble`, старые `go2_deploy/install`, simulation `lo`, domain 1 | Чистый Jazzy underlay; только собственные Jazzy overlays; real NIC; domain согласовать с Go2 (старое real-решение использует 0) |
| `Dockerfile`: `ros:humble-ros-base`, `ros-humble-*` | Отдельный Jazzy/Jetson профиль с согласованным CUDA runtime либо native Jazzy; не выбирать образ только заменой строки distro |
| Gazebo Classic зависимости в Dockerfile | Убрать из real-профиля; они не нужны для SDK2/RL/Point-LIO/RARS |
| Vendored `rmw_cyclonedds` из Humble | Не использовать как Jazzy overlay; использовать установленный Jazzy RMW |
| `rosidl_generator_dds_idl` и старый DDS IDL generator в vendored messages | Использовать уже адаптированные Jazzy `unitree_go/unitree_api` из navigation; установленный `ros-jazzy-rosidl-generator-dds-idl` не найден |
| ROS1 `roscpp/rospy` в package.xml | Удалить ошибочные export dependencies; объявить фактические ROS2 зависимости |
| Старые `tf2_geometry_msgs/*.h` при переносе nav-патчей | Сохранить Jazzy `.hpp` и существующие API-адаптации |
| Hardcoded `/home/ruben/libtorch`, `find_library` вручную | Внешний `Torch_DIR`; linkage через CMake Torch config, ABI flags и корректный RPATH |
| C++14 в RL package | Установленный Torch target требует **C++20**; также текущий код использует `std::filesystem` |
| SDK2 `link_directories(.../x86_64)` | Imported SDK2 target с `${CMAKE_SYSTEM_PROCESSOR}`; aarch64 DDS libs |
| Безусловная сборка MuJoCo/GLFW/LCM/legacy SDK | Real-only build по умолчанию; simulator и SDK examples отдельно/выключены |

**Дубли пакетов:** `unitree_go/unitree_api` есть и внутри `workhop_rl/src/unitree_ros2/cyclonedds_ws/src/unitree`, и в `autonomy_nav_go2/src/utilities/unitree_pkgs`. Для real выбрать единственного владельца — Jazzy-пакеты navigation. Исключить старые copies и вложенный Humble RMW из discovery/build, не менять reference.

`unitree_sdk2` в vendored дереве имеет CMake version `2.0.0`; aarch64 SDK archive и ELF ARM64 `libddsc.so/libddscxx.so` присутствуют. Их наличие не означает успешную линковку/совместимость на данной системе. Upstream также указывает поддержку aarch64, но prebuild environment — Ubuntu 20.04/GCC 9.4: нужен локальный ABI/link check. [Unitree SDK2 README](https://github.com/unitreerobotics/unitree_sdk2).

SDK2 и ROS CycloneDDS держать в отдельных процессах по решению `go2_nav`. В real ROS-контроллере SDK headers/linkage для RobotState не нужны; служебный mode-switch — SDK2-only. Не добавлять vendored DDS directory глобально в `LD_LIBRARY_PATH` ROS-процесса: это может подменить Jazzy DDS.

### Torch / CUDA: важная поправка к старому `go2_nav`

Фактический доступный Torch prefix:

```text
/home/ruben/RARS_sdk_grasp_net/rars01_graspnet/.venv/lib/python3.12/site-packages/torch
```

Этот путь использовался только для аудита; целевой runtime должен иметь документированный собственный environment. В CMake установленного Torch обнаружено требование C++20. CUDA compiler и Torch CUDA version совпадают по `13.2`.

**Одного `CMAKE_CUDA_ARCHITECTURES=87` недостаточно:** установленный `Caffe2/public/cuda.cmake` явно предупреждает, что этот параметр игнорируется и надо использовать `TORCH_CUDA_ARCH_LIST`. Для Orin-профиля выставить `TORCH_CUDA_ARCH_LIST=8.7`, проверить фактическую компиляцию и GPU операции. Не считать old `go2_nav` CMake полностью готовым.

RL сейчас создаёт CPU tensors и загружает policy без GPU placement. Сначала проверить CPU inference budget 20 ms; перенос RL на CUDA — отдельная оптимизация с согласованным device для модели и всех observations. GraspNet CUDA worker оставить отдельным процессом; проверить нагрузку одновременно с Point-LIO и RL, а не только изолированный forward.

В рабочей копии `rars01_graspnet` отсутствуют `.venv`, `sdk/graspnet-baseline` и соседний `sim2real/repos/rars_arm_sdk`. Модели в `models/` присутствуют, но CUDA extensions, Orbbec, RARS Python module и checksum всех grasp weights в этом аудите не валидировались. Нельзя считать внешний установленный проект полноценной зависимостью нового workspace.

## 6. Интеграция с `autonomy_nav_go2:ros2_Jazzy`

Целевая цепочка:

```text
Go2 /utlidar/cloud + /utlidar/imu
  → transform_sensors → Point-LIO → /state_estimation + /registered_scan
  → terrain analysis → FAR → /way_point → localPlanner → /path
  → pathFollower → /cmd_vel (TwistStamped, frame vehicle)
  → navigation gate + watchdog → unified RL 50 Hz
  → единственный real Go2 LowCmd publisher

RARS SDK feedback + фактически принятый arm target
  → RARS ROS bridge → шесть q, dq, target для RL
  → отдельный RARS command loop 100 Hz / STM32
```

| Интерфейс | Обязательное условие |
|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped`, физические единицы, scaling ровно один раз в Agent |
| `/navigation_active` | `std_msgs/Bool`, transient-local QoS; default false; выключение блокирует locomotion request |
| `/lowstate` | `unitree_go/LowState`; finite/fresh IMU и моторы; steady-clock receive watchdog |
| `/state_estimation` | `nav_msgs/Odometry` от Point-LIO; реальные stamps и валидный TF |
| `/registered_scan` | `PointCloud2`; сохранить ring/time поля и единицы для deskew |
| `/utlidar/transformed_raw_imu` | Для Point-LIO с ускорением; не legacy relay с обнулённым acceleration |
| RARS observation | Ordered `joint1..joint6`, rad/rad/s, measured feedback; stamp, readiness, accepted target |
| TF | Один владелец каждого transform; IMU-centred pose и base pose не смешивать |
| Actuation | `sendSportCommand=false`; не запускать `vel_ctrl_repub`/Sport controller параллельно RL |

Существующий `system_real_robot_rl_navigation.launch` полезен, но **с текущим workhop controller несовместим**: передаёт параметры, которые тот не объявляет/не использует, и ожидает `/cmd_vel`-вход. `autostart=false` в этом launch сам по себе не отключает текущий publisher.

### Найденные navigation-разрывы

- `stage4d_full_navigation.launch.py` — sim-only: `use_sim_time=true`, MuJoCo, виртуальные arm/payload механизмы. Его нельзя запускать как real launch.
- В Jazzy отсутствуют `utlidar_sim.yaml`, `far_planner/config/sim_pointlio.yaml`, `graph_decoder/launch/decoder.launch`; последний заменён на `decoder.launch.py`. Старый Stage4D launch ссылается на отсутствующие файлы.
- `mapping_utlidar.launch` задаёт `use_imu_as_input=false`, а `utlidar.yaml` задаёт `true`. Выбрать один проверенный режим, убрать противоречие и проверить effective parameters.
- `transform_everything.py` по умолчанию ищет `~/Desktop/imu_calib_data.yaml`, при ошибке использует defaults. Для real запускать с явным проверенным calibration path и диагностикой валидности.
- В Jazzy sensor bridge нет sim-параметра `preserve_sensor_stamp`; он вводит общий offset после первого LiDAR packet. Нужно проверить совместимость временных шкал реального IMU/cloud/Point-LIO, а не переносить sim calibration.
- Default `sensorOffsetX=0.3` в real navigation launch отличается от дипломного IMU-centred bridge. Реальный offset измерить; симуляционный `-0.02557` автоматически не использовать.
- Jazzy branch потеряла часть дипломных изменений: localPlanner diagnostics/cache/replanning, pathFollower motion model, FAR path validation; удалён `path_validation.h` и его test. Нужен выборочный перенос соответствующих алгоритмов из read-only reference с сохранением Jazzy API fixes.
- В Jazzy local planner launch жёстко задано `vehicleLength=0.3`, `vehicleWidth=0.7`; это не доказанная collision envelope Go2+RARS01. Нужен измеренный footprint и проверка вращения/выноса руки; старые параметры standalone Go2 недостаточны.
- `/path` должен принадлежать localPlanner; включённый Point-LIO path remap-ить в отдельное имя. Не создавать вторую локализацию через ground truth/simulation bridges.

RARS01 real driver работает через отдельный `rars_arm_sdk`, а не Go2 LowCmd. Существующий `stage4d_real_grasp_planner.py` — **offline simulation adapter** с `OfflineArm`; его название не означает управление реальной рукой. Сохранить IK/trajectory reuse, добавить настоящий транспорт, единственного владельца serial и interlock «манипуляция при подтверждённой остановке базы». `/navigation_active=false` само по себе не является подтверждением физической остановки.

## 7. Конкретные файлы будущего изменения

Пути ниже относительны к `sim2real/repos/`. Это перечень планируемых изменений; сейчас ни один из этих файлов не изменён. Новые имена — предлагаемые артефакты реализации.

### Обязательно: `workhop_rl`

| Файл | Изменение |
|---|---|
| `env_setup.sh`, `real_setup.sh` | Убрать Humble/старые overlays; real Jazzy environment |
| **Новый** `jazzy_setup.sh` | Адаптированный setup из go2_nav с параметризованным Torch/NIC |
| `Dockerfile`, `README.md` | Отдельный native/Jetson real recipe и запрет reference runtime |
| `src/unitree_rl_controller-ros2/CMakeLists.txt` | C++20, внешний Torch, ABI/export dependencies, сохранить unified tests |
| `src/unitree_rl_controller-ros2/package.xml` | Убрать ROS1 exports, объявить rclcpp/yaml и фактические зависимости |
| `src/unitree_ros2_to_real/CMakeLists.txt` | Real-only targets, SDK2 aarch64, отдельный mode-switch, Torch/RPATH, dependency/link cleanup |
| `src/unitree_ros2_to_real/package.xml` | ROS2 runtime/build deps, std_srvs/diagnostics/ament index по реализации |
| `src/unitree_ros2_to_real/include/ros2_rl_go2.hpp` | CmdVel/navigation/arm interfaces, freshness/readiness, state machine |
| `src/unitree_ros2_to_real/src/ros2_rl_go2.cpp` | Unified paths, arm observations, mapping всех режимов, reset, output gate, watchdog, faults, nonblocking startup |
| `src/unitree_ros2_to_real/include/navigation_command_adapter.hpp`, `src/navigation_command_adapter.cpp` | Параметризованные real velocity limits при сохранении contract semantics |
| `src/unitree_ros2_to_real/include/motor_crc.h`, `src/motor_crc.cpp` | Проверить packet layout/packing/CRC; менять только при выявленном несовпадении |
| **Новые** `src/unitree_ros2_to_real/include/go2_motion_mode.hpp`, `src/go2_motion_mode.cpp`, `src/go2_mode_switch.cpp` | Выборочный перенос SDK2-only tooling из go2_nav |
| **Новый** `src/unitree_ros2_to_real/config/go2_rars01_real.yaml` | Unified contract + real thresholds; явно выбрать policy |
| **Новый** `src/unitree_ros2_to_real/launch/go2_rars01_real.launch.py` | Jazzy real wiring, default disarmed, use_sim_time=false, без симулятора |
| **Новые** `src/unitree_ros2_to_real/scripts/rars01_real_bridge.py`, `config/rars01_real_bridge.yaml` | Обёртка existing RARS driver; feedback/accepted target/status и interlocks |
| **Новые/расширяемые** `src/unitree_ros2_to_real/tests/real_controller_contract_test.cpp`, `tests/navigation_command_adapter_test.cpp` | Mapping, переходы, faults, freshness, запрет output до arming; transport mock |
| **Новый** `.colcon/real.meta` | Явно исключить nested Humble RMW, duplicate interfaces, simulator/examples из real build |

Также **новый** `sim2real/weights/POLICY_MANIFEST.yaml`: SHA, contract, training provenance и deployment profile. Существующий `policy_2.pt` не менять.

### Обязательно/по выборочному переносу: `autonomy_nav_go2`

| Файл | Изменение |
|---|---|
| `src/base_autonomy/vehicle_simulator/launch/system_real_robot_rl_navigation.launch` | Подключить unified real controller/bridge либо делегировать orchestration новому launch; не создавать два RL nodes |
| `src/base_autonomy/local_planner/launch/local_planner.launch` | Footprint/TF ownership, подтверждённые limits и параметры recovered algorithms |
| `src/base_autonomy/local_planner/src/localPlanner.cpp`, `src/pathFollower.cpp` | Выборочно вернуть дипломные planner/replanning/motion-model изменения; сохранить Jazzy headers и gates |
| `src/base_autonomy/local_planner/CMakeLists.txt`, `package.xml` | Зависимости diagnostics/visualization после backport |
| `src/slam/point_lio_unilidar/launch/mapping_utlidar.launch` | Убрать конфликт режима IMU; явный calibration path; отдельный path topic |
| `src/slam/point_lio_unilidar/config/utlidar.yaml` | Только измеренные real extrinsics/time/IMU settings |
| `src/utilities/transform_sensors/transform_sensors/transform_everything.py` | Явная calibration validity/time diagnostics; расширить self-filter под Go2+RARS01 при проверке облака |
| `src/route_planner/far_planner/include/far_planner/{far_planner.h,graph_planner.h,node_struct.h,planner_visualizer.h,utility.h}` | Review и выборочный backport дипломных FAR behavior changes |
| `src/route_planner/far_planner/src/{far_planner.cpp,graph_planner.cpp,planner_visualizer.cpp,utility.cpp}` | Соответствующий FAR backport |
| **Восстановить** `src/route_planner/far_planner/include/far_planner/path_validation.h`, `tests/path_validation_test.cpp` | Path-validation logic и проверка из reference |
| `src/route_planner/far_planner/CMakeLists.txt`, `package.xml`, `launch/far_planner.launch.py` | Tests/dependencies/config integration с сохранением Jazzy launch |
| **Новый** `src/route_planner/far_planner/config/go2_rars01_real.yaml` | Отдельный real planner profile вместо sim_pointlio.yaml |

Полный перенос FAR/localPlanner файлов не назначается автоматически: diff между reference и Jazzy содержит и алгоритмические, и distro-изменения. Каждый behavioral patch должен сохранять Jazzy-адаптацию и проходить offline regression.

### RARS и description

- **Новый** `rars01_graspnet/config/go2_sim2real.yaml`: локальные SDK/URDF/calibration пути и bridge profile с наследованием проверенных defaults.
- `rars01_graspnet/README_JETSON_ORIN_NANO.md`: воспроизводимая подготовка именно `sim2real`; добавить pin соседнего `rars_arm_sdk`.
- **Новый внешний компонент** `sim2real/repos/rars_arm_sdk` с зафиксированным SHA: без него configured real arm transport отсутствует.
- `rars01_description`: обязательных изменений URDF сейчас не выявлено. При несовпадении измеренной hardware geometry нужны отдельные правки соответствующих URDF, а не изменение RL порядка суставов.
- `rl_agent.cpp`: математику сохранять; возможная небольшая техническая правка loading/eval/inference mode после измерения C++ inference, без изменения observation/action semantics.

## 8. Порядок переноса и критерии готовности

| Этап | Работа | Критерий завершения |
|---|---|---|
| 1. Зафиксировать входы | Manifest новой policy, training metadata, SHA всех repos и RARS SDK | Известно, что именно переносится; неизвестные training параметры отмечены явно |
| 2. Изолировать Jazzy build | Setup/CMake/package.xml, единственные interfaces, SDK2 aarch64, real-only targets | Сборка только из sim2real, без Humble/reference overlays |
| 3. Проверить offline RL | C++ загрузка policy, CPU timing, Python↔C++ parity, reset/history/action mapping | `[1,315]→12`; одинаковые observations/actions; устойчивый бюджет 20 ms |
| 4. Проверить SDK/packet | Link/ABI, CRC golden fixtures, все motor slots и mode transitions | Корректная сериализация без подключения actuator transport |
| 5. Добавить RARS bridge | SDK module, measured q/dq, accepted target, freshness и serial ownership | Никаких подмен measured state; готовность руки явно участвует в RL gate |
| 6. Восстановить nav поведение | Выборочный backport, real footprint, TF/time/calibration | Offline/replay regression; единственный `/path`, корректный TwistStamped |
| 7. Проверить read-only real inputs | Только sensor/feedback consumers; publisher LowCmd отключён | Реальные топики/QoS/stamps/оси/порядок суставов подтверждены |
| 8. Проверить Jetson совместную нагрузку | CUDA ops/GraspNet worker, Point-LIO + RL timing/RAM | Нет пропусков RL deadline и конфликтов Torch/DDS/runtime |
| 9. Отдельный actuator commissioning | Последующий самостоятельный этап после устранения блокеров | Определённые переходы/остановка, проверенная firmware ownership, контролируемая аппаратная проверка |

**До real actuation должны быть закрыты:** model/config selection, mapping stand-up/hold, реальные arm observations, reset при re-entry, startup output gate, freshness/fault handling, валидный пакет/CRC, единственный владелец привода, TF/calibration и runtime/build isolation.

**Статус этого аудита:** статический разбор и CPU binary check завершены. Jazzy C++ build, CUDA runtime, ROS graph/QoS на роботе, аппаратные калибровки и устойчивость policy на настоящем Go2 пока не подтверждены. Эти проверки перечислены как следующие этапы, а не как уже выполненные результаты.
