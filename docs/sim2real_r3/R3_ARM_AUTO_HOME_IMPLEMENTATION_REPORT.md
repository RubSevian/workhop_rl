> Коррекция 04.10.2026: по уточнению оператора feedback этой STM появляется после enable. Прежняя зависимость countdown от feedback удалена; streaming в initial grace запускается без fabricated measurements. Проверки исправления описаны в R3_AUTO_HOME_ENABLE_ORDER_FIX.md.

# R3 RARS01 AUTO HOME — отчёт реализации

Дата: 04.10.2026. Jetson Orin Nano / aarch64 / ROS 2 Jazzy.

**Реализован отдельный постоянный serial owner: SDK serial/receiver connection → 10 с → один SDK enable → непрерывный HOME для всех семи моторов → честный arm_home_ready. Go2 takeover только читает readiness руки. Сборка и все 17 offline-тестов прошли. Физическое удержание HOME пока не проверено.**

Код реализован по `CODEX_R3_RARS01_AUTO_HOME.md`. Его требование gripper=0 заменяет прежнюю идею захватывать измеренную позицию gripper при старте.

## SDK и калибровка

SDK: `RubSevian/rars_arm_sdk`, ветка `main`, SHA **f90278b46125f2b311e4555173321e80a6c7be3f**. Исходники SDK не изменены; рабочее дерево SDK чистое.

Owner использует только существующие connect/enable/sendPositionTargets/tryReadJointState/communicationStatus/lastError. `setZero()` не вызывается. В тестируемом control interface вообще нет API изменения нулей или калибровки. Existing directions/offsets загружаются из выбранного SDK config без перезаписи файла. Преобразование моторных координат остаётся внутри SDK; второго преобразования q/dq нет.

## Startup FSM и задержка

```text
WAIT_DEVICE
→ подключение с retry (existing port_retry_interval_s, иначе 1 с)
→ STARTUP_DELAY: 10 с непрерывно usable feedback
→ SDK enable один раз
→ HOLD_HOME

runtime fault → FAULT_LATCHED
```

Standalone owner по умолчанию `read_only=true`, `connect_serial=false`. Подготовленная boot-служба явно выбирает физический профиль `read_only=false`, `connect_serial=true`; deployment config содержит `auto_home.enabled=true`. Это исключает неожиданное enable при обычном диагностическом запуске owner без аргументов.

Задержка начинается после успешного SDK serial/receiver connection. Motor feedback до enable не требуется: по уточнению оператора эта STM отдаёт его после enable. Потеря соединения до enable сбрасывает countdown. Отсутствующий USB приводит к безопасным повторным попыткам connect; enable не вызывается. Время кнопки питания Jetson не угадывается.

SDK enable содержит свою protocol delay около 200 мс и сброс статистики receiver. Owner после enable сразу запускает HOME stream, одновременно ожидая новый feedback; initial feedback grace берётся из existing SDK config, если задан. Во время ожидания readiness false; ни нули вместо измерений, ни фиктивные timestamps не создаются.

## HOME и частота потока

```text
HOME_TARGET = [0,0,0,0,0,0,0]
```

Первые шесть значений — рука; последнее — gripper. Startup не захватывает текущую позицию gripper. Согласно заданию он также физически командуется в calibrated zero. Положение вне HOME tolerance снимает readiness, но не меняет заданный HOME на текущую позу.

Existing `robot.rars01.command_rate_hz` загружается в `ArmConfiguration.command_rate_hz`; deployment `auto_home.command_rate_hz` может явно переопределить его. Текущее значение 100 Гц. ROS timer и controller используют эту частоту. Controller сохраняет сетку периодов, пропускает опоздавшие slots без burst отправок; реальная частота/джиттер требуют физического измерения.

Поток не зависит от Sport, stand, RL или остановки leg node. SDK enable сам по себе скрытого hold loop не создаёт — sendPositionTargets выполняет owner.

## Определение arm_home_ready

Readiness true только когда одновременно:

- owner находится в HOLD_HOME;
- SDK connected и enabled_local;
- есть свежий полный feedback, common age ≤ 0,25 с;
- все семь valid flags и IDs 1..7 корректны;
- все семь измеренных q/dq конечны;
- все семь motor status равны enabled/normal=1; disabled=0 не допускается;
- SDK feedback watchdog и опубликованный STM watchdog не tripped;
- последняя успешная отправка HOME не старше target timeout 0,25 с;
- абсолютная ошибка каждого из семи measured q относительно HOME ≤ 0,15 рад.

До enable motor feedback может отсутствовать — это не блокирует startup и не является аппаратным диагнозом. Это не означает HOME ready. Для enabled motor feedback после enable предусмотрен ограниченный grace; затем отсутствие enabled/normal фиксируется как fault.

При отсутствии первого feedback q/dq публикуются неизвестными, не fabricated zero. Диагностика содержит owner_state, connected, enabled_local, enable_attempted, command_rate_hz, feedback_age, motor_id[7], motor_status[7], valid7, q[7], dq[7], home_error[7], HOME target, target age/valid, watchdog flags, protocol_v2_detected, arm_home_ready и last_error.

## Accepted target и RL observation

Accepted timestamp обновляется **только после успешного SDK send**. Ошибка отправки не обновляет timestamp, сразу делает target_valid и arm_home_ready false и фиксирует fault. Задержка потока больше target timeout также фиксирует fault до следующего send, поэтому пропущенный поток не маскируется новой отправкой.

Actor получает measured q/dq только joint1..6. Accepted `q_arm_des=[0,0,0,0,0,0]`; gripper исключён из actor, но физически командуется owner в zero. SDK successful send подтверждает software control path, а не аппаратный echo всех targets; HOME tracking проверяется дополнительно по измерениям.

## Интеграция с пультом Go2

```text
L1+L2+A
→ существующие R3 prerequisites + arm_home_ready
→ если false: Sport release не запрашивается, LowCmd ownership не создаётся
→ если true и остальные gates закрыты: существующая автоматическая цепочка ног
```

Добавлен явный blocker `arm_home_not_ready`. Leg node проверяет HOME diagnostics всех семи моторов, включая gripper, и freshness feedback/accepted target. Legacy status без HOME поля не проходит новую deployment gate.

Из leg node удалена публикация запросов arm HOLD_CURRENT: takeover, abort и emergency не заменяют постоянный HOME руки. Он не вызывает arm enable/setZero/sendPositionTargets. Owner больше не имеет старого arm hold_current service/subscription, способного включать руку вне startup FSM.

Новые автоматические leg движения не добавлены; mapping/policy math и остальные physical gate flags не изменены. Автоматический возврат в Sport, IK/GraspNet trajectories и изменения навигации остаются вне задачи.

## Fault, перезапуск и single owner

До SDK enable атомарно сохраняется durable ENABLE_ATTEMPT с boot ID. Проблема записи запрещает enable. FAULT записывается отдельно и сохраняется между boot; штатная попытка прошлого boot допускает следующий обычный boot. Повторный запуск после enable attempt в том же boot запрещён и консервативно становится persistent fault.

Runtime disconnect, stale/invalid feedback, motor fault, watchdog, локальный disable, отсутствие physical enabled после grace, send failure и stale stream приводят к FAULT_LATCHED. Owner остаётся жив для диагностики, targets больше не отправляются, автоматического повторного enable нет. Recovery не реализует автоматическое удаление journal.

Сохраняется existing per-device serial lease, добавлена стабильная deployment-wide owner lease: позднее появление by-id symlink не создаёт второй owner. Все cooperating процессы должны использовать общий lock directory. SDK GUI и direct GraspNet backend, не соблюдающие lease, необходимо исключить из параллельного serial доступа. ОС не заставляет чужую программу соблюдать advisory lock.

Для будущего IK/grasp этот же owner остаётся serial backend; второй control процесс или исполнение траекторий сейчас не создаются.

## Boot / systemd

Подготовлены `deployment/rars01-owner.service`, env example, executable launcher и README установки. Unit включается в обычную загрузку через multi-user.target после операторского deployment. Он имеет `Restart=no`, persistent StateDirectory `/var/lib/rars01-owner` и shared RuntimeDirectory `/run/rars01-owner`; journal не находится в volatile /run.

Config path задаётся явно; optional verified stable device path переопределяет существующий configured port. Выдуманного by-id пути и нового жёстко заданного ttyACM0 нет. Launcher использует только Jazzy overlay этого workspace.

**Unit подготовлен, но не установлен/включён/запущен в этой работе.** Его реальный запуск автоматически включает моторы после задержки, поэтому commissioning руки проводится отдельно. Intentional owner shutdown использует existing SDK destructor, который best-effort отключает моторы; остановка только leg node на owner не влияет.

Подробности установки и recovery: `src/unitree_ros2_to_real/deployment/README.md`.

## Изменённые файлы

В `workhop_rl/src/unitree_ros2_to_real`:

- `include/rars_auto_home.hpp`, `src/rars_auto_home.cpp`: testable FSM, transport interface, readiness и durable journal.
- `src/rars_r3_owner.cpp`: SDK adapter, saved config, retry, timer, seven-motor diagnostics, single owner.
- `include/r3_commissioning.hpp`, `src/r3_commissioning.cpp`: HOME gate перед release/output.
- `src/go2_r3_commissioning.cpp`: HOME IPC validation, actor six-joint accepted target, удаление leg arm-control requests, readiness/error diagnostics.
- `config/go2_rars01_real.yaml`: AUTO HOME, 10 с, семь zero targets, 0,15 рад, 100 Гц и required HOME gate.
- `CMakeLists.txt`: core/tests и установка deployment с executable permissions.
- `tests/rars_auto_home_test.cpp`, `tests/rars_auto_home_isolation_test.py`, `tests/r3_commissioning_test.cpp`: mock и integration regression.
- `deployment/`: service/env/launcher/README и versioned snapshot root workspace helpers.

Root `sim2real/scripts/verify_r3.py` обновлён для новых read-only defaults; его версия включена в deployment workspace_scripts snapshot. Task и этот отчёт сохранены также в `workhop_rl/docs/sim2real_r3`.

## Проверки

- Финальная Jazzy/aarch64 сборка: **5 packages PASS**, 37,6 с. Только существующие CMake предупреждения Torch/CUDA, C++ compile errors отсутствуют.
- Полный offline regression: **17 tests, 0 errors, 0 failures, 0 skipped**.
- Mock AUTO HOME: до 10 с нет enable; enable once; reconnect/countdown reset; семь нулей включая gripper; continuous stream; configured 50/100 Гц; real measured q сохраняются; stale/disabled/fault/bad ID/NaN/Inf/send failure/watchdogs; post-enable feedback grace; persistent journal/restart.
- Leg tests: HOME false запрещает release; HOME true при остальных ready gates позволяет обычный release; потеря HOME между запросом и переходом блокирует release.
- Isolation checks: отсутствует setZero; направления/offsets загружаются без изменения; leg не посылает targets/enable; service Restart=no.
- Observation parity: frame/history max absolute error `8e-7`, PASS.
- CPU 500 samples: JIT max 13,847 мс, agent max 11,255 мс. Это не заменяет длительный timing под полной perception нагрузкой.
- Python AST, shell syntax, `git diff --check`: PASS.
- `systemd-analyze verify`: exit 0; предупреждения относятся к двум уже установленным NVIDIA units с устаревшим syslog output, не к новому unit.
- SDK SHA и чистое рабочее дерево проверены повторно.

Workspace логи: `build_r3_auto_home_final.log`, `test_r3_auto_home.log`, `systemd_rars01_owner_verify.log`. Regression log также сохранён в `docs/sim2real_r3/test_r3_auto_home.log`.

## Что ещё требует физической проверки

1. После enable + запуска HOME stream должны прийти все семь достоверных feedback в configured grace. До enable feedback для этой STM не требуется. Полнота/свежесть после enable по-прежнему нуждается в физической проверке.
2. Calibrated zero всех семи моторов, включая физическое закрытое положение gripper, реальные motor IDs/status и HOME tracking.
3. Фактические 100 Гц, target/feedback ages, SDK/STM watchdog и поведение при пропаже USB/команд. Для protocol v1 отсутствие STM watchdog flag не доказывает проверенную работу watchdog; protocol_v2_detected публикуется отдельно.
4. Freshness каждого отдельного CAN мотора: `per_joint_freshness_proven=false` сохранён, поскольку common USB frame age не является доказательством индивидуальной CAN freshness.
5. Длительный leg timing с облаками/навигацией/GraspNet и прежние physical gates поддержки Go2, emergency gains и lie-down.

В этой работе не запускались serial connect, motor enable, HOME streaming на железе, SDK Sport release, LowCmd или движение. **Физический HOME hold не заявляется подтверждённым.**

## Коммиты и push

Ветка `ros2_go2_rars01_real`.

- Предыдущий незакоммиченный R3 сохранён checkpoint-коммитом `96c49c9`: remote commissioning + SDK DDS isolation.
- AUTO HOME, boot files, tests и этот report сохраняются следующим отдельным commit с сообщением `feat: add persistent RARS01 auto-home owner and takeover readiness`.

Push не выполняется. После проверки отправить ветку:

```bash
git -C ~/go2_diploma/sim2real/repos/workhop_rl push -u origin ros2_go2_rars01_real
```
