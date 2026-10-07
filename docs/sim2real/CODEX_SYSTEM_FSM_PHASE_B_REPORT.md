# System FSM — PHASE B

Дата: 05.10.2026. Approval: [CODEX_SYSTEM_FSM_PHASE_B_APPROVAL.md](CODEX_SYSTEM_FSM_PHASE_B_APPROVAL.md). Baseline39b78c9.

**Код PHASE B реализован в отдельном candidate `build_fsm/install_fsm`. 27/27 tests PASS (26 regression + отдельный CPU benchmark), A → RL byte-identical с baseline.**

**Offline CPU timing PASS:** после подтверждения остановки physical leg launch выполнен `policy_cpu_test`: JIT max15,27 мс, полный шаг agent max2,43603 мс при бюджете20 мс. Это synthetic compute benchmark, не длительное измерение системы с LiDAR/навигацией/физическим IO. `install_r1` и текущие процессы не заменены.

Новая lie-down динамика и physical arm emergency disable не commissioned; flags остаются false. NAV/FULL не активируются без настоящих adapters. Команды запуска и новые кнопки: [README](../../README.md).

## Принятые ограничения

NAV clamp/finite/freshness сохранится. Lie-down target APPROVED; trajectory duration/gains/tolerance/timeout/settle остаются предметом commissioning. Stand0,02/6/4 с, gains40/1 и RL25/1, actor315-D/mapping, tickets/watchdog, packet/CRC, Sport helper, leases и SDK calibration не меняются.

На начало работы были запущены прежние физические node ног и arm owner; агент их не останавливал и не заменял. После подтверждения оператора «Тест завершён, launch ног остановлен» host process check не обнаружил controller ног, затем выполнен offline CPU benchmark. Все новые функциональные FSM tests — synthetic/fake, без motor commands.

## Артефакты

- tests/fixtures/r3_baseline: test-only baseline namespace/include snapshot, не runtime FSM.
- system_fsm_baseline_trace: одинаковые fake IO/orchestration/input clocks, byte-level packets/CRC, reset и first-policy events.
- build_fsm/install_fsm: отдельная Jazzy/aarch64 Release сборка, CUDA compiler указан явно, arch87/Torch8.7.

Первый configure не нашёл CUDA compiler/architecture автоматически; повтор выполняется с теми же explicit native CUDA args, что у baseline. Runtime behavior этим не меняется.

## Шаг1 — PASS

Отдельная clean Release сборка завершилась (runtime/fsm_step01_build.log). Frozen baseline regression и byte-identical differential oracle2/2 PASS (runtime/fsm_step01_test.log). Полные runtime sources прежние; изменены только test targets и docs.

## Шаг 2 — профили

Семь строгих launch-профилей и таблица capabilities; неизвестное имя отклоняется. Legacy flags имеют явное отображение. Build PASS; parser/capabilities и frozen differential: 3/3 PASS. Runtime подключается в последующих шагах.

## Шаг 3 — readiness

Разделены HOME для takeover и arm_control_ready для runtime. Navigation/perception/emergency validation — отдельные факты, по умолчанию false. Тест здоровой руки вне HOME и blockers: PASS; build и 4/4 targeted tests PASS.

## Шаг 4 — центральный владелец

R3Supervisor хранит один SystemState (7 значений) и внутреннюю phase. Старый R3State вычисляется для совместимости, отдельного mutable state_ больше нет. Dispatch — единственное место записи global state; B/X/A имеют порядок приоритета. Build и 6/6 тестов PASS, включая прежнюю regression и byte differential.

## Шаг 5 — A / capabilities

Startup parser вызывается до загрузки policy и любых физических адаптеров. Legacy launch отображается в immutable profile; конфликт явного profile/flags отклоняется. Services проверяют capabilities/readiness; LEG_SAFETY заканчивает captured hold без stand/RL. Новый A dispatch использует прежний Sport/lease/graph/capture/stand/hold/reset. Build PASS; 7/7 targeted tests PASS, byte-identical A baseline подтверждён. NAV readiness не выводится из cmd_vel.

## Шаг 6 — X

ACTIVE + X закрывает velocity gate и оставляет zero RL до принятого HOME-запроса и свежего подтверждения HOME/settle. Handoff: fresh measured q, первый fixed packet без скачка; lie-down завершается постоянным SYSTEM_HOLD. HOME timeout оставляет RL; invalid/stale/policy fault — central emergency. Lie timeout удерживает последний planned target (после полной интерполяции — approved lie_down_q), output/lease остаются. Dynamics gate false сохраняет zero RL и explicit blocker. Build/8 targeted tests PASS; A differential не изменился. ROS arm IPC подключается в шаге 8, candidate не готов к физическому запуску до завершения wiring.

## Шаг 7 — A из SYSTEM_HOLD

Restart требует свой healthy lease/publisher и fresh RELEASED, захватывает новые measured q и переиспользует output. Ни release RPC, ни новый enable не запрашиваются. Тест полного restart проверяет capture, прежние 6/4 с, один reset и RL entry; build/9 targeted tests PASS, initial A baseline byte-identical.

## Шаг 8 — B и async ports

B защёлкивает EMERGENCY_FAULT даже до output; при eligibility выдаёт прежние leg 0/3, инвалидирует policy и HOME, не ждёт arm RPC. Async callbacks проверяют orchestration generation. HOME/emergency проходят через прежнего единственного RARS owner. HOME idempotent, без enable; arm disable запрещён по умолчанию и выполняется один раз только при emergency_disable_validated, с прекращением HOME stream до SDK disable. Accepted RPC не равен physical disable. Реальные NAV/manip cancellation adapters пока отсутствуют: status unavailable, profiles не получают fake readiness.

Первый compile выявил требования Jazzy к явным callback types; исправлено, повтор build PASS. 11/11 targeted tests PASS: B во всех stop phases, до publisher, приоритет, stale callbacks/policy, no recovery, single-owner HOME/emergency mocks; initial A differential PASS.

## Шаг 9 — runtime HOME blocker

ACTIVE использует arm_control_ready + прежние реальные freshness/target checks; HOME/static_hold остаются обязательны для takeover и PD/SYSTEM_HOLD. Away-HOME при healthy control не fault, в том числе FULL_MISSION fake fixture. Контрольная потеря health всё ещё central emergency. Build и 12/12 targeted tests PASS, A oracle PASS.

## Шаг 10 — status / launch / config

Статус state теперь global (7 состояний), phase — internal, legacy_state — compatibility projection. Добавлены effective profile/capabilities, control readiness, ports/blockers, generation и target. Все launch capability/config flags имеют read-only descriptors; OpaqueFunction не подставляет конфликтующие legacy defaults. Maintenance services и executor actions проходят Dispatch.

Approved target записан в production и operator trial copies. Dynamics остаются false; arm emergency validation false. Optional settle принимает 0; чужой lie target не выдаётся за approved. README обновлён для candidate install_fsm и реальных ограничений NAV/FULL. Build/15 targeted tests PASS. Launch test первоначально пытался писать ROS logs вне sandbox; log directory перенесён в temporary writable directory, повтор PASS.

## Шаг 11 — итоговая проверка

- Release/Jazzy/aarch64 build PASS; отдельная установка candidate.
- Финальный review убрал ROS graph/service queries из 500Гц Refresh: диагностические snapshots обновляются на прежних 50Гц, актуальный publisher/lease отдельно проверяется на A restart edge. После изменения regression и ROS smoke повторно PASS.
- Regression **26/26 PASS** плюс отдельно **policy_cpu_test 1/1 PASS**, всего **27/27** на той же candidate сборке. Включены actor315-D/reset/mapping, packet/CRC, SDK DDS isolation, RARS HOME и все System FSM tests.
- CPU: один intra/inter-op thread, CPU actor `[1,315] → [1,12]`; по50 warmup и500 measured samples для JIT и полного agent. JIT mean1,06489 / p95 1,79222 / p99 5,5221 / max15,27 мс; agent mean1,34663 / p95 1,53616 / p99 1,72123 / max2,43603 мс. Порог20 мс и watchdog не изменены. Эти ~2,2 с теста не подтверждают длительную частоту всей системы под нагрузкой.
- A differential сравнивает каждый сериализованный LowCmd, gains/dq/tau/CRC, legacy transition ticks, reset и first accepted policy events. SHA256 trace: `0544cc26a41fcad7eae80108f5c12e18a879f1aab42e193beb213740bbb21262`.
- Виртуальные IO500Гц/policy50Гц: 8/30/100/200/1000мс, single pending job/no backlog, прежние response/job deadlines и burst. Исправлена неточность первоначальной synthetic fixture (первый job/float clock); runtime thresholds не менялись.
- X: HOME timeout/отказ сохраняет zero RL; invalid/stale/control/policy failure явным fault. In-flight job после measured handoff отвергается. Повторный X в SYSTEM_HOLD сохраняет этот hold. Прямой maintenance PD hold с рукой вне HOME отклоняется, RL продолжает работать.
- A restart: reuse own output, fresh measured q, no release/enable RPC, stand6/hold4/один reset. B: все takeover/stop/hold phases, priority/latch/no recovery, zero/3 eligible packets, single-owner gated emergency.
- Profiles: сервисные Dispatch gates и staged ACTIVE sessions не позволяют RL_ZERO получать ненулевую скорость или менять REMOTE/NAV source.
- ROS smoke — domain223, только loopback, read_only=true. Реальные NAV callback: clamp, expiry, stale/future/zero/malformed stamp и NaN. Enable/stand/RL refused; runtime profile/legacy params upgrade refused; X/B events распознаны, **sent_packets=0, LowCmd publishers=0**.
- Unknown profile startup (`bad`, `REMOTE_TEST`) отказал до model load/physical adapters. `verify_r1/r2/r3` PASS в чистом Jazzy окружении.
- Runtime actor/core, safety_io/MakeLowCmd/CRC, Sport helper, SDK calibration, planner и reference не изменены. Git status NAV/SDK/reference чистый.

### Логи

Все находятся в `/home/ruben/go2_diploma/sim2real/runtime/`:

| Артефакт | Результат |
|---|---|
| `fsm_step01..11_build.log` | Сборки перед соответствующими commits |
| `fsm_step11_test.log` | 17/17 targeted tests PASS |
| `fsm_full_regression.log` | 26/26 regression PASS |
| `fsm_cpu_timing.log` | CPU benchmark1/1 PASS, JIT max15,27 мс, agent max2,43603 мс |
| `fsm_baseline_parity_final.log` | Exact A trace SHA |
| `fsm_readonly_ros_smoke.log` | ROS loopback/NAV/immutable params/zero output PASS |
| `fsm_unknown_startup.log` | Unknown startup failure PASS |
| `fsm_verify_clean.log` | verify_r1/r2/r3 PASS |

Ранее был обнаружен старый controller PID12383; агент его не останавливал. После подтверждения оператора host process check не обнаружил controller ног; только после этого запущен CPU benchmark. Новых физических LowCmd, Sport release/enable, serial enable/targets/disable и команд движения не отправлял. AUTO HOME/emergency проверялись mock transport; в ROS smoke arm owner не запускался.

## Что осталось до physical commissioning

1. Offline CPU benchmark завершён PASS. Перед commissioning оценить длительную нагрузку всей системы, особенно после подключения LiDAR/навигации; короткий synthetic benchmark не заменяет эту проверку.
2. Отдельно проверить approved lie-down trajectory с реальной рукой/payload: duration/gains/tolerance/timeout/settle. Target уже approved; dynamics gate пока false, X остаётся в zero RL с blocker.
3. Bench-проверить SDK arm disable через того же owner. До этого gate false, B legs работают по прежней eligibility, arm disable не подтверждён.
4. Подключить реальные NAV/manipulation/perception adapters отдельным этапом. До их появления NAV/FULL readiness=false; cancellation unavailable не выдаётся за успех.

Команда воспроизведения уже выполненного **offline** CPU timing (только после остановки physical leg launch):

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /home/ruben/go2_diploma/sim2real/setup.bash
source /home/ruben/go2_diploma/sim2real/install_fsm/local_setup.bash
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ctest --test-dir /home/ruben/go2_diploma/sim2real/build_fsm/unitree_legged_real \
  -R '^policy_cpu_test$' -V
```

Это тест вычислений, без ROS publisher/serial/Sport. Измеренный результат сохранён в `runtime/fsm_cpu_timing.log`; превышения в последующих тестах нельзя лечить изменением watchdog в рамках этого refactor.

## Изменённые runtime файлы

Пути относительно `src/unitree_ros2_to_real/`:

| Файлы | Назначение |
|---|---|
| `include/src operation_profile` | Parser, immutable capabilities, legacy mapping |
| `include/src system_readiness`, `system_state` | Facts, семь состояний, events/phases/ports |
| `include/r3_commissioning.hpp`, `src/r3_commissioning.cpp` | Один global owner, A/X/B, HOME-to-PD handoff, persistent hold |
| `src/go2_r3_commissioning.cpp` | Events, async clients/generation, readiness/status/immutable params |
| `include/rars_auto_home.hpp`, `src/rars_auto_home.cpp`, `src/rars_r3_owner.cpp` | Idempotent HOME/gated disable у одного owner |
| `launch/go2_rars01_r3_commissioning.launch.py` | Профили без конфликтующих legacy defaults |
| `config/go2_rars01_real.yaml`, `config/profiles/go2_rars01_commissioning.yaml` | Approved target и не commissioned критерии/gates |
| `scripts/r3_manual_lib.py`, workspace smoke/verify helpers, `CMakeLists.txt` | Compatibility и проверки |

Добавлены test-only frozen fixtures, differential/scheduler/profile/readiness/X/A/B tests. README и общий журнал решений обновлены; корневые workspace copies синхронизированы.

## Коммиты и push

Ветка: `ros2_go2_rars01_real`. Первые десять commits:

```text
ad1659d test: freeze validated Go2 baseline and differential oracle
5ae69ff feat: define immutable operation profiles and capability limits
3f891f3 feat: split system takeover and runtime readiness facts
7a66a94 refactor: centralize seven-state system dispatch ownership
3c8e516 refactor: gate takeover and services by immutable system capabilities
2dbd058 feat: orchestrate zero-RL HOME handoff and persistent lie-down hold
5503486 feat: restart from system hold with fresh capture and reused output
9cdcb9c feat: latch priority emergency and wire gated single-owner arm ports
59e160c fix: require healthy arm control instead of HOME during active RL
ba4d6ef feat: expose system status and immutable launch profiles with approved stop config
```

Шаг 11 — финальный regression/timing fixtures и этот отчёт, отдельный заключительный commit. Изменения не пушились. Команда для оператора:

```bash
git -C /home/ruben/go2_diploma/sim2real/repos/workhop_rl push origin ros2_go2_rars01_real
```
