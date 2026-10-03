# Sim2Real Phase R1 — результат реализации

**3 октября 2026 · Jetson Orin Nano · Ubuntu 24.04 · ROS 2 Jazzy · aarch64**

## Результат

**R1 выполнена: изолированная Jazzy-сборка прошла для пяти пакетов, шесть C++ тестов прошли.** Новая policy загружается C++ runtime и выдаёт конечный `[1,12]` из `[1,315]`. CPU baseline укладывается в 20 ms в выполненных измерениях.

`ros2_rl_go2` теперь представляет диагностический **DISARMED** runtime. В его сборке **вообще нет LowCmd publisher, motor transport, SDK2 ChannelFactory, serial transport или arming API**. Загрузка модели, приход feedback или navigation command не включают выход. `enable_actuator_output=true` отвергается. Автоматический stand-up отсутствует.

Реальные stand/hold/RL transitions подготовлены в отдельном transport-free core и проверены на синтетических данных. Это подготовка контроллера, а не разрешение управлять настоящим роботом. Actuator transport и RARS bridge остаются задачами R2.

**Не выполнялись:** ROS nodes/launch, публикация LowCmd, переключение Sport Mode, serial connect, motor enable и движение Go2/RARS01. SDK mode-switch только собран, ни разу не запущен. Read-only reference не изменялся и его Humble install/build не использовались. Навигационные алгоритмы не переносились.

## Репозитории

| Репозиторий | Ветка | HEAD | Изменения R1 |
|---|---|---|---|
| workhop_rl | `ros2_go2_rars01_real` | `b5085333029f8174d11884fcd457ddbde04ad6c8` | Рабочие изменения, без commit |
| autonomy_nav_go2 | `ros2_Jazzy` | `a5f3e67f8116394fbbd2e3364ca9e8e9fa2cd2c9` | Нет |
| rars01_graspnet | `jetson` | `8efeb6ffaa042617f07e32c5f2dff939d8241b73` | Нет |
| rars01_description | `go2_arm` | `2df2bf4f9a28437124b4eedf6e1777c2f497b709` | Нет |
| rars_arm_sdk | `main` | `f90278b46125f2b311e4555173321e80a6c7be3f` | Нет; только инспекция |

SDK-only источники выборочно взяты из локального `origin/go2_nav` на `e20e8ad659dd2a1ec58ffacf536eeb856f4ba254`. Полного merge не было.

## Что сохранено

Без изменения исходников/математики сохранены:

- `unified_observation_contract.hpp/.cpp`;
- `rl_agent.cpp`, включая unified frame, previous action, clipping/scaling/default pose;
- исходный `go2_rars01_unified.yaml`;
- `navigation_command_adapter.hpp/.cpp` и его watchdog/navigation semantics;
- существующие observation/history/reset/config tests и Python parity reference.

Real profile содержит ту же секцию `go2_rars01`, что unified profile. `verify_r1.py` проверяет их полное YAML-равенство. Real limits и таймауты вынесены отдельно в `real_deployment`; перед command adapter применяются deployment velocity limits. В Agent масштабирование выполняется ровно один раз.

Mapping **motor → policy**: `[3,4,5,0,1,2,9,10,11,6,7,8]`. Policy order — FL/FR/RL/RR; motor order — FR/FL/RR/RL. Core использует mapping для measured q/dq и всех stand/hold/RL target arrays, kp/kd и torque-limit metadata. Проверены разные значения gains/limits и асимметричные hip signs.

На каждом offline входе HOLD → RL вызывается текущий `ResetPolicyState()`: previous action обнуляется, первый актуальный frame повторяется пять раз. Проверены первый и повторный входы с изменённым arm observation.

## Точные изменённые и добавленные файлы

Пути этой таблицы относительны к `sim2real/repos/workhop_rl/`. **9 изменённых + 11 новых файлов.**

| Файл | Статус | Назначение |
|---|---|---|
| `README.md` | Изменён | R1-инструкции и отделение старого Humble/sim recipe |
| `env_setup.sh` | Изменён | Делегирование чистому Jazzy setup |
| `real_setup.sh` | Изменён | Удаление старых Humble/DDS/install путей |
| `jazzy_setup.sh` | Новый | Внешний Torch_DIR, Jazzy RMW, R1 overlay, отказ от смешанных/неожиданных overlays |
| `src/unitree_rl_controller-ros2/CMakeLists.txt` | Изменён | C++20, внешний Torch, RPATH, регистрация сохранённых CTest tests |
| `src/unitree_rl_controller-ros2/package.xml` | Изменён | Удаление ROS1 exports, явные rclcpp/yaml-cpp зависимости |
| `src/unitree_ros2_to_real/CMakeLists.txt` | Изменён | Real-only build, transport-free core, SDK-only C++17 targets, native SDK2 import, private DDS RPATH, offline tests |
| `src/unitree_ros2_to_real/package.xml` | Изменён | Валидные фактические ROS2 dependencies без redundant/ROS1 exports |
| `src/unitree_ros2_to_real/include/ros2_rl_go2.hpp` | Изменён | Диагностический ROS interface без actuator members |
| `src/unitree_ros2_to_real/src/ros2_rl_go2.cpp` | Изменён | DISARMED startup, explicit paths, fail-closed loading, TwistStamped/navigation gate, finite/fresh LowState diagnostics, один CPU thread |
| `src/unitree_ros2_to_real/include/real_controller_core.hpp` | Новый | Mapping, readiness/output gate, offline mode/target API |
| `src/unitree_ros2_to_real/src/real_controller_core.cpp` | Новый | Strict real config, measured mapping, stand/hold interpolation, повторный RL reset и CPU inference без транспорта |
| `src/unitree_ros2_to_real/config/go2_rars01_real.yaml` | Новый | Неизменённый unified contract + отдельные real deployment thresholds |
| `src/unitree_ros2_to_real/include/go2_motion_mode.hpp` | Новый | SDK-only mode API из go2_nav |
| `src/unitree_ros2_to_real/src/go2_motion_mode.cpp` | Новый | SDK-only RobotState tooling из go2_nav; не исполнялся |
| `src/unitree_ros2_to_real/src/go2_mode_switch.cpp` | Новый | Отдельный mode-switch executable; не исполнялся |
| `src/unitree_ros2_to_real/tests/real_controller_contract_test.cpp` | Новый | Gate/readiness, mapping, transitions, reset/re-entry, fail-closed model/config loading |
| `src/unitree_ros2_to_real/tests/policy_cpu_test.cpp` | Новый | C++ policy shape/finite checks и CPU timing |
| `src/unitree_ros2/COLCON_IGNORE` | Новый | Исключение nested Humble RMW и дублирующих Unitree interfaces/examples |
| `src/unitree_mujoco/COLCON_IGNORE` | Новый | Исключение симулятора из стандартного real discovery |

Новые файлы относительно `sim2real/`:

| Файл | Назначение |
|---|---|
| [scripts/build_r1.sh](scripts/build_r1.sh) | Изолированная сборка пяти явно выбранных package roots, один compiler job |
| [scripts/test_r1.sh](scripts/test_r1.sh) | Только offline tests, результаты CTest и Python/C++ parity |
| [scripts/verify_r1.py](scripts/verify_r1.py) | SHA policy, равенство unified config, отсутствие actuator paths в R1 source, discovery isolation |
| [weights/POLICY_MANIFEST.yaml](weights/POLICY_MANIFEST.yaml) | Идентичность policy, contract, CPU profile, явное отсутствие training provenance |
| [PHASE_R1_IMPLEMENTATION_REPORT.md](PHASE_R1_IMPLEMENTATION_REPORT.md) | Этот отчёт |

Policy binary, исходный audit и файл задания не менялись. Build/install/log directories — generated artifacts. Основные результаты: [build log](build_r1_console.log), [test log](test_r1_console.log), `build_r1/`, `install_r1/`, `log_r1/`, `log_r1_test/`. Логи package discovery находятся в `log_r1_discovery/` и `log_r1_list/`.

## Environment и overlay order

Использованы Ubuntu 24.04/aarch64, GCC 13.3, JetPack `7.2.1-b49`, CUDA toolkit `13.2.86`, native Torch `2.13.0+cu132` с C++11 ABI. Сборка и C++ inference успешно подтвердили совместимость используемого Torch с текущим компилятором/линковкой.

Внешний Torch CMake:

```text
/home/ruben/RARS_sdk_grasp_net/rars01_graspnet/.venv/lib/python3.12/site-packages/torch/share/cmake/Torch
```

Это **явно переданная build/runtime dependency**, а не автоматически выбранный fallback. Для автономной доставки R2 потребуется закрепить/воспроизвести этот environment.

Порядок сборки/зависимостей:

```text
/opt/ros/jazzy
  → install_r1/ros2_unitree_legged_msgs
  → install_r1/unitree_api + install_r1/unitree_go
  → install_r1/unitree_rl_controller
  → install_r1/unitree_legged_real
```

Фактический `AMENT_PREFIX_PATH` после setup, в порядке поиска:

```text
sim2real/install_r1/unitree_legged_real
sim2real/install_r1/unitree_rl_controller
sim2real/install_r1/unitree_go
sim2real/install_r1/unitree_api
sim2real/install_r1/ros2_unitree_legged_msgs
/opt/ros/jazzy
```

`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `TORCH_CUDA_ARCH_LIST=8.7`. DDS robot NIC, Sport Mode и arming setup не настраиваются. ROS_DOMAIN_ID остаётся значением окружения/default ROS; его реальное сетевое согласование относится к R2.

`unitree_go/unitree_api` имеют единственного владельца: Jazzy sources из `autonomy_nav_go2`. Целиком navigation workspace не собирался и его install не source-ился. `colcon list` на выбранных roots показывает ровно пять ament packages; старый nested RMW, simulator, RARS SDK и Unitree examples туда не входят.

SDK2 использует aarch64 static archive и aarch64 DDS libraries. SDK targets собираются с C++17: старые DDS-CXX headers не принимаются GCC 13 в C++20. Torch/core/ROS targets — C++20. Vendor headers не менялись.

SDK-only `go2_mode_switch` имеет `RUNPATH=$ORIGIN/sdk2`; private DDS libraries установлены рядом в `lib/unitree_legged_real/sdk2`. Эта директория не экспортируется в ROS `LD_LIBRARY_PATH`. `readelf` подтвердил отсутствие SDK2/direct DDS linkage у ROS executable и отсутствие Torch/ROS linkage у mode-switch.

Несмотря на CPU inference, CUDA-built Torch линкует `libtorch_cuda.so`. Наличие этой зависимости не означает GPU inference. `nvcc` нужен для CMake detection такого Torch. CUDA runtime/GPU execution в R1 не проверялись.

## Выполненные команды

Из `/home/ruben/go2_diploma`:

```bash
Torch_DIR=/home/ruben/RARS_sdk_grasp_net/rars01_graspnet/.venv/lib/python3.12/site-packages/torch/share/cmake/Torch \
  bash sim2real/scripts/build_r1.sh > sim2real/build_r1_console.log 2>&1

Torch_DIR=/home/ruben/RARS_sdk_grasp_net/rars01_graspnet/.venv/lib/python3.12/site-packages/torch/share/cmake/Torch \
  bash sim2real/scripts/test_r1.sh > sim2real/test_r1_console.log 2>&1
```

Build script содержит точную expanded команду `colcon build`: явные пять roots, `build_r1`, `install_r1`, sequential executor, `MAKEFLAGS=-j1`, Release, `BUILD_TESTING=ON`, `BUILD_LEGACY_SIM=OFF`, явные policy/config paths, CUDA compiler `/usr/local/cuda/bin/nvcc`, architecture 8.7.

Test script выполняет:

```bash
python3 scripts/verify_r1.py
colcon --log-base "$R1_ROOT/log_r1_test" test \
  --build-base "$R1_ROOT/build_r1" --install-base "$R1_ROOT/install_r1" \
  --base-paths repos/workhop_rl/src/unitree_rl_controller-ros2 repos/workhop_rl/src/unitree_ros2_to_real \
  --packages-select unitree_rl_controller unitree_legged_real \
  --executor sequential --event-handlers console_direct+ --ctest-args -V
colcon test-result --test-result-base "$R1_ROOT/build_r1" --verbose
python3 repos/workhop_rl/tests/reference_unified_observation.py \
  --cpp-test "$R1_ROOT/build_r1/unitree_rl_controller/unified_observation_contract_test"
```

Это команды внутри test script после перехода в `sim2real` и Jazzy setup. Ни одна команда запуска hardware executable не входит в scripts.

В процессе подготовки устранены два build failures: не найденный CUDA compiler (теперь задаётся явно) и C++20 template-destructor ошибки SDK DDS headers (SDK-only targets переведены на C++17). Последняя сборка: **5 packages finished**, exit 0. Optional Kineto/CUDA architecture warnings предыдущего configure не являлись ошибками; Torch detection показал flags `compute_87/sm_87`.

## Проверки и CPU timing

| Проверка | Результат |
|---|---|
| `unified_observation_contract_test` | PASS: frame layout, dimensions, gravity, history order/action timing, clipping |
| `agent_unified_reset_test` | PASS: repeated current frame, zero previous action |
| `agent_config_loading_test` | PASS: сохранённые legacy/unified config behaviors |
| `real_controller_contract_test` | PASS: disarmed/gates, incomplete readiness, fault latch, mapping/gains/limits, stand/hold transitions, RL re-entry, missing/corrupt model/config |
| `navigation_command_adapter_test` | PASS: navigation gate, clipping, watchdog, NaN не обновляет timestamp; assertions включены и в Release |
| `policy_cpu_test` | PASS: C++ TorchScript load, `[1,315]→[1,12]`, конечные zero/test outputs, Agent action dimension, CPU budget |
| Python/C++ frame/history parity | PASS; max absolute error `8e-07` для frame и history |
| `verify_r1.py` | PASS: policy SHA, config parity, default output disabled, source transport isolation |
| `git diff --check` | PASS |

`colcon test-result`: **6 tests, 0 errors, 0 failures, 0 skipped**. Parity и verification script — дополнительные offline проверки, не включённые в эти шесть CTest tests.

Policy SHA-256:

```text
a70b18b88e6ee2fe32de4904559404f4f672dff2396e95ca2c851fdefc5a8cdc
```

CPU measurements: один intra-op и один inter-op thread, 50 warm-up calls, 500 measured calls на каждую строку, Release. ROS runtime использует те же thread settings.

| CPU путь | Mean ms | p50 ms | p95 ms | p99 ms | Max ms |
|---|---:|---:|---:|---:|---:|
| TorchScript `[1,315] → [1,12]` | 2.11535 | 1.34171 | 5.11242 | 7.79792 | 10.4901 |
| Agent frame + history + action | 2.39427 | 1.81127 | 5.28538 | 6.47855 | 7.58204 |

Оба измеренных пути ниже 20 ms во всех 500 samples. Это CPU baseline без полного навигационного/GraspNet workload; real-time deadline под совместной нагрузкой и thermal throttling ещё не подтверждён. GPIO/ROS transport и аппаратный цикл не измерялись.

Startup проверен transport-free тестами и анализом compiled source/build dependencies; живой ROS node специально не запускался в соответствии с ограничениями фазы. В R1 невозможно включить output через ROS parameter/service/key: такой API отсутствует.

## RARS SDK: результат инспекции

`rars_arm_sdk:main`, SHA `f90278b46125f2b311e4555173321e80a6c7be3f`.

- C++17 target `rars_arm_sdk`, alias `rars_arm::sdk`; основной API — `include/rars_arm.hpp` / `RarsArm`.
- Python — опциональный pybind11 module `rars_arm_py` из `python/bindings.cpp`, включаемый `RARS_ARM_BUILD_PYTHON=ON`.
- Qt hardware example по умолчанию включён; для будущей headless bridge build выключать `RARS_ARM_BUILD_QT_EXAMPLE`.
- Зависимости: Threads, LibSerial; для bindings — Python development files и pybind11. На машине обнаружены `libserial-dev 1.0.0-9build1`, `python3-dev 3.12.3-0ubuntu2.1`, `pybind11-dev 2.11.1-2`.
- `try_read_joint_state()` возвращает новый feedback либо None. C++ `tryReadJointState()` переводит raw motor coordinates: `q_joint = direction × (q_raw − zero_offset)`, `dq_joint = direction × dq_raw`.
- Индексы 0..5 — рука, 6 — gripper. Доступны measured position/velocity/torque, `valid`, motor IDs/status, `communication_status()` с feedback age/watchdog flags.
- `send_position_targets`, `send_configured`, `connect/enable/disable/set_zero` — аппаратные APIs; ни один из них не вызывался и SDK instance не создавался.

Полная bridge не реализована. Future readiness должна подтверждать шесть валидных measured joints и их свежесть; общий feedback age нельзя автоматически считать доказательством свежести каждого сустава без проверки пакетов/агрегации на STM32. Target для RL должен отражать реально принятый arm command, а не подменять measured feedback.

## Что остаётся для R2 и проверки на физическом Go2

1. **RARS bridge:** serial ownership, шесть q/dq, accepted arm target, timestamps/validity/watchdog, joint signs/zeros, interlocks базы и руки. В R1 arm readiness намеренно false; реальные arm observations не выдумываются.
2. **Actuator transport:** точный LowCmd layout/header/unused slots/CRC golden fixtures для фактических interfaces/прошивки. Старые `motor_crc.*` сохранены, но не входят в R1 real target. Корректность физического packet path пока не заявляется.
3. **Ownership/stop semantics:** проверить Sport Mode state/status и единственного владельца привода на данной прошивке; определить физическое поведение при timeout/fault/прекращении LowCmd. Старые SDK comments не заменяют проверку реального firmware.
4. **Arming и физические limits:** explicit operator arming, подтверждённая stance readiness, аппаратные q/gain/torque/rate limits, fall/fault handling. Mapped torque limits в offline Targets — metadata, не hardware torque limiter.
5. **RL provenance:** training commit/checkpoint/run остаются неизвестны; подтвердить массу/инерции Go2+RARS01, payload/latency/randomization и arm-target semantics новой policy.
6. **Навигация:** отдельная фаза выборочного дипломного backport поверх Jazzy, реальный footprint, TF/calibration/time/QoS. В R1 поддерживается будущий `TwistStamped`/navigation gate interface, но сами navigation nodes не изменялись/не запускались.
7. **Jetson environment:** закрепить автономный native Torch/GraspNet/RARS environment; проверить CUDA runtime и совместную нагрузку Point-LIO/GraspNet/RL, RAM/thermal behavior.
8. **Read-only hardware verification:** фактический порядок motors/arm joints, quaternion convention/IMU axes, units/stamps и соответствие measured poses URDF. Mapping уже доказан программными тестами; аппаратное соответствие ещё надо измерить.

Старые simulation launch files и Humble Dockerfile сохранены как legacy исходники и не входят в default R1 runtime. Для R2 использовать новый real orchestration после устранения перечисленных аппаратных неопределённостей.
