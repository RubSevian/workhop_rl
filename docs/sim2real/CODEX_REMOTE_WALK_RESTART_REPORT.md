# REMOTE walking и restart inference — 07.10.2026

## Что изменено

По запросу оператора REMOTE command limits: vx[-0.5,0.5] м/с, vy[-0.5,0.5] м/с, yaw[-0.5,0.5] рад/с. Сохранены mapping ly/-rx/-lx, deadband0.01, finite/source/profile/X gates. Это диапазоны команд в policy, не измеренное ограничение фактической скорости.

Добавлен отдельный startup YAML `r3_commissioning.remote.command_limits` в production и operator trial, совместимый fallback к старым manual limits при отсутствии ключа. NAV/manual0.2/0.1/0.1 и NAV clamp/freshness неизменны. Статус публикует remote_command_limits и navigation_command_limits.

## Откуда могла взяться ошибка restart

Лог /home/ruben/.ros/log/go2_r3_commissioning_12047_1791367515059.log: первый RL около80 с → успешный X/HOME/lie/PASSIVE/OFF → повторный stand6/hold4 → первый Act221.556 мс. На42.066 мс сработал in-flight watchdog; затем Sport observation age501.954 мс прекратил eligibility damping. policy_ms4.88 — предыдущий завершённый вызов.

**Найден конкретный runtime дефект:** в go2_r3_commissioning startup/worker вызовы Agent не имели `torch::InferenceMode`, хотя offline CPU tests его использовали. Проверка сохранённого policy_2.pt: все8 параметров requires_grad=true, output requires_grad=true (AddmmBackward0). Сам eval не отключает autograd.

Agent помещает предыдущий action в каждый63-D frame; history clones/cat сохраняют связи autograd. Поэтому без guard новая policy output связана с цепочкой прошлых forward, а не только с фиксированными5 frame значениями. `output_dof_pos` также сохраняет предыдущий graph; при новом Act после reset его замена может освобождать длинную старую цепочку непосредственно внутри измеренного compute. **Это обоснованная причина возможной задержки и роста памяти; точный источник всех221 мс в физическом инциденте без stack/CPU trace не доказан.** Возможны также JIT перепрофилирование после смены requires_grad при reset, scheduling/CPU power/прочие задержки. Sport status result обрабатывается в начале PolicyTick; заблокированный Act задерживает обработку async SDK результата, что может объяснить вторичное устаревание Sport. Первый fault в логе всё равно policy_inference_timeout, не Sport.

Исправлен именно runtime контекст: InferenceMode в warmup и отдельно в каждом policy executor callback, охватывает подготовку observations, reset, history, forward и target extraction. Guard thread-local, поэтому одного guard в constructor недостаточно для ROS worker threads. Actor/core SDK source, weights, mapping,315-D contract, history/reset math, stand/gains и watchdogs не менялись. Порог40 мс не увеличивался.

## Проверки

**PASS: согласованная Release/Jazzy/aarch64 сборка candidate install_fsm,32/32 tests, read-only ROS loopback smoke и verify_r1/r2/r3.**

- Actual C++ Agent: без guard выход требует grad, история начинает хранить autograd со второго frame; с guard output/action/history не требуют grad.
- Numerical parity30 frames между guarded/unguarded policy, including commands±0.5: allclose rtol1e-5/atol1e-6 PASS.
- Worker thread,5 sessions по200 frames с reset/паузами: никаких grad graphs, max first Act2.45929 мс. Это synthetic проверка, не полное воспроизведение physical80 с/idle/restart/ROS.
- CPU500 samples: JIT max1.06554 мс, полный agent max1.54313 мс при20 мс.
- Remote bounds±0.5/clamp/deadband, X/source/zero-profile gates и прежний NAV bounds PASS. Initial A byte differential и A/X/B/PASSIVE restart regressions PASS.
- ROS smoke: startup с новым guard, finite/NAV freshness/clamp, immutable profile, X/B и zero LowCmd publishers PASS.

Логи runtime/remote_inference_build.log, remote_inference_regression.log, remote_inference_restart.log, remote_inference_ros_smoke.log, remote_inference_verify.log. Первоначальный test assertion ожидал grad history уже на первом frame, где previous_action ещё нулевой; исправлена fixture, исходный отказ сохранён в remote_inference_initial_fixture.log. Runtime defect и numeric parity подтверждены финальными тестами.

Физический controller ног при проверке отсутствовал, HOME owner PID11632 продолжал работать; агент его не трогал. Offline вычисления не посылают LowCmd/SDK/serial commands. Повторный physical RL/restart после исправления ещё не выполнялся агентом.

## Запуск с пультом

HOME owner уже запущен отдельно; второй owner или SDK GUI одновременно не запускать. Новый leg executable берётся из candidate install_fsm, нужен новый launch.

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

В другом терминале с тем же candidate окружением:

```bash
ros2 run unitree_legged_real r3_status.py --timeout 3600
# или полный сырой payload:
ros2 topic echo /go2/locomotion_status --full-length
```

До A нужны fresh LowState/Sport и arm_control_ready/arm_home_ready. A (L1+L2+A ≥0.75 с, отпустить) автоматически release Sport при необходимости → measured hold → stand6 → hold4 → RL. Нейтраль стиков даёт zero velocity. ly вперёд/назад, -rx боковое движение, -lx yaw; каждый канал±0.5. Затем малые отклонения стиков и проверка motion_command/remote_command_limits.

X: zero RL/HOME → measured PD → lie-down/settle → PASSIVE10 → output OFF/SYSTEM_HOLD. B: прежний emergency active mode1 kp0/kd3, без HOME wait. После X — повторный A, проверить, что первый RL job после повторного подъёма принят и нет policy_inference_timeout. При новом fault сохранить полный status/transition log; не увеличивать watchdog для маскировки задержки.

## Файлы и сохранение

go2_r3_commissioning.cpp (worker/startup guard, bounds status), r3_commissioning.hpp/cpp (separate remote limits), production/trial YAML, CMake и два новых tests: remote_walk_limits_test, runtime_inference_restart_test; статический node wiring test, README/общий журнал решений. Workspace trial YAML/README/report синхронизированы. Изменения пока не закоммичены/не запушены.
