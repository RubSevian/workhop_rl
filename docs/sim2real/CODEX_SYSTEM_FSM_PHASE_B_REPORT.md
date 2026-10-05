# System FSM — PHASE B

Дата: 05.10.2026. Approval: [CODEX_SYSTEM_FSM_PHASE_B_APPROVAL.md](CODEX_SYSTEM_FSM_PHASE_B_APPROVAL.md). Baseline39b78c9.

Статус: реализация продолжается, шаг1 — freeze/differential oracle. Runtime physical install_r1 не заменяется; candidate build_fsm/install_fsm.

## Принятые ограничения

NAV clamp/finite/freshness сохранится. Lie-down target APPROVED; trajectory duration/gains/tolerance/timeout/settle остаются предметом commissioning. Stand0,02/6/4 с, gains40/1 и RL25/1, actor315-D/mapping, tickets/watchdog, packet/CRC, Sport helper, leases и SDK calibration не меняются.

На начало работы ещё запущены прежние физические node ног и arm owner. Их не останавливаю и не заменяю. CPU/timing benchmark отложен до подтверждения завершения physical leg launch. Все новые функциональные FSM tests — synthetic/fake, без motor commands.

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
