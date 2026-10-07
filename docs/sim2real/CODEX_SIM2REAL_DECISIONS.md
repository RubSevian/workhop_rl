# Sim2Real Go2 + RARS01: актуальные решения

Обновлено: 05.10.2026. Порядок запуска: [README.md](../../README.md).

Это краткое состояние проекта. Новые решения вносятся в соответствующие разделы; устаревшая хронология не накапливается.

## Среда и источники

- Jetson Orin Nano, aarch64, ROS 2 Jazzy.
- Workspace: `/home/ruben/go2_diploma/sim2real`; рабочая ветка `workhop_rl:ros2_go2_rars01_real`.
- Sim2Sim reference — read-only; его Humble build/install не используется в runtime.
- MuJoCo reference задаёт actor contract, stand pose, gains и длительности; `workhop_rl:go2_nav` — прежнюю real-robot/Jazzy логику.
- Навигация: `autonomy_nav_go2:ros2_Jazzy`; рука: `rars01_graspnet:jetson`; описание: `rars01_description:go2_arm`.

## Принятая схема управления

| Вход | Поведение |
|---|---|
| **L1+L2+A ≥0,75 с** | Проверка HOME → освобождение Sport → measured hold → подъём → удержание → RL |
| **L1+L2+X ≥0,75 с** | Прекратить RL и удерживать измеренную позу; автоматического lie-down нет |
| **L1+L2+B ≥0,75 с** | Пассивный emergency damping; fault latch, без автоматического возврата RL |

После X новый A явно повторяет цикл, без второго publisher и повторного release уже освобождённого Sport. После B/fault восстановление отдельно. B имеет высший приоритет; комбинации требуют отпускания перед повтором. Автоматического возврата в Stock нет.

## Длительности и gains

Источник: reference `mujoco_sim.cpp`, policy_dt=0,02, линейная интерполяция до 300 тиков, standup_done после 500 тиков.

| Этап | Длительность | Kp / Kd всех 12 моторов |
|---|---|---|
| Начальный captured hold | Один 20 мс такт | 40 / 1 |
| Линейный подъём | 6 с | 40 / 1 |
| Удержание перед RL | 4 с | 40 / 1 |
| RL | До выхода/fault | 25 / 1 |
| X: measured hold | До новой команды | 40 / 1 |
| B: passive damping | До отдельного восстановления | 0 / 3 |

Stand заканчивается по времени. Stand tracking gate и проверка отклонения от первого captured hold на 0,01 рад удалены; ошибки позы остаются диагностикой. Поле hold_capture_tolerance_rad удалено из startup-лога и статуса. Допуск для проверки сформированных пакетов и отдельной ручной lie-down процедуры не является условием старта подъёма. B задаёт dq=0, tau=0; это не траектория к lie-down target. Fixed/RL gains и default pose точно совпадают с unified Sim2Sim config.

## Источники команд policy

- **remote_test:** workshop `vx=ly`, `vy=-rx`, `wz=-lx`; пределы 0,20 / 0,10 / 0,10. `/cmd_vel` игнорируется.
- **autonomy:** только `/cmd_vel`, `geometry_msgs/msg/TwistStamped`; стики не вмешиваются. В pathFollower `sendSportCommand=false`.
- **motion_commands_enabled=false:** выбранный источник скорости выключен, policy получает нулевую скорость. A/X/B остаются действующими.
- Старые manual staging/deadman не подключены; manual_step отклоняется.

## Рука

Один отдельный serial owner: connect → 10 с → один enable → непрерывный HOME всех семи моторов `[0,0,0,0,0,0,0]` на 100 Гц → валидный feedback → readiness. Feedback до enable не требуется. Actor использует шесть суставов, захват исключён.

HOME tolerance=0,15 рад, freshness=0,25 с. SDK calibration/directions/gains сохранены. A проверяет HOME, X/B не меняют задания руки. GUI и другой direct serial backend одновременно с owner не используются. Journal только записывает enable/fault; прежняя запись не блокирует новый запуск. После fault текущий процесс прекращает stream и не выполняет повторный enable. Явный новый запуск owner начинает новую сессию с задержкой10 с. Systemd unit подготовлен; установка/включение не входит в текущий ручной запуск.

Физические HOME-тесты ранее подтвердили примерно 100 Гц и arm_home_ready; это не физический PASS ног/RL.

## Контракт и защита

- Observation: 63 значения × 5 кадров = 315; actor output — 12 leg actions.
- Первый вход RL сбрасывает previous action и history из текущих измерений, с нулевой командой скорости.
- Policy порядок FL/FR/RL/RR; hardware FR/FL/RR/RL. Перестановка `[3,4,5,0,1,2,9,10,11,6,7,8]` применяется один раз в каждую сторону.
- Unitree quaternion wxyz→xyzw; gyro xyz соответствует reference.
- SDK2 mode helper работает в отдельном процессе. LowCmd появляется после RELEASED и проверки отсутствия чужого publisher; используется output lease и SDK CRC.
- A/X/B обрабатываются в независимом IO callback, без ожидания завершения Act(). Mutex сериализует смену режима, результат policy и отправку пакета.
- X/B/fault инвалидируют policy ticket; старый результат не возобновляет RL. Дедлайны ответа policy и свежесть входов разделены: принятие результата обновляет watchdog ответа; LowState и arm snapshots проверяются отдельно. Invalid action и watchdog остаются проверяемыми.
- Навигация отвергает negative/overflow/zero/future/stale timestamps и NaN/Inf без завершения node. QoS depth=1, timeout=0,25 с.

## Проверенный результат

Последняя повторная проверка исправила зависимость обработки кнопок от callback policy и исключение при некорректном nav timestamp.

| Проверка | Результат | Артефакт в `runtime/` |
|---|---|---|
| Jazzy/aarch64 build/install | PASS | `build_r3_reaudit_final.log` |
| Controller offline tests | 12/12 PASS | `test_r3_reaudit_final.log` |
| Actor contract/reset/config tests | 3/3 PASS | `test_rl_contract_reaudit.log` |
| ROS remote_test, loopback/read-only | PASS | `smoke_r3_reaudit_remote.log` |
| ROS autonomy, malformed input cases | PASS | `smoke_r3_reaudit_autonomy.log` |
| Policy/config parity и static validators | PASS | `verify_r3_reaudit.log` |

Проверены цикл 6+4 с, X при подъёме/первом inference, удержание всех 12 measured q, отбрасывание результата после X, B=0/3 и отсутствие возврата после fault. ROS-тесты: domain223/loopback, sent_packets=0, LowCmd publishers=0. Физические команды при последнем audit не отправлялись.

## Что ещё не подтверждено

- Физическая работа последнего исправления watchdog policy. Последний присланный запуск прошёл подъём и HOLD, вошёл в RL_ZERO, затем остановился по policy_result_stale; разбор и исправление ниже. Новый физический запуск не выполнялся.
- Полный timing PASS: ранее CPU max=26,0426 мс при бюджете20 мс. По текущей просьбе длительную обработку повторно не оценивали; runtime guards сохранены.
- Независимая свежесть радиопульта: SDK даёт wireless_remote в LowState без отдельного RF timestamp. ОС/executor не дают гарантированного аппаратного времени реакции.
- B — passive damping; отдельная геометрическая lie-down trajectory и её physical gate не подтверждены.
- IMU-калибровка навигации, LiDAR и полноценная работа руки/навигации — следующие этапы.

Production config сохраняет неподтверждённые допуски закрытыми. `config/profiles/go2_rars01_commissioning.yaml` — отдельно принятый оператором экспериментальный профиль; его флаги не являются измеренным PASS.

Сырые логи, capture и `runtime/first_physical_audit/` сохранены. Старые дублирующие отчёты удалены. Исходные задания фаз в `repos/workhop_rl/docs/` сохранены как требования, не как инструкция запуска.

## Разбор физического запуска 05.10.2026, timestamp 1791185347

По присланным оператором строкам:

| Переход | Timestamp | Результат |
|---|---|---|
| HOLD_CURRENT | 1791185347.449134 | Первый пакет отправлен, fault отсутствует |
| STAND_TRANSITION | 1791185347.506583 | Подъём начался |
| HOLDING | 1791185353.507704 | Подъём завершился за 6,001 с |
| RL_ZERO | 1791185357.526744 | Удержание длилось 4,019 с; RL включился с нулевой скоростью |
| EMERGENCY_DAMP | 1791185357.553589 | Через 26,846 мс после входа RL: policy_result_stale |

Это не остановка по допуску первого hold и не отказ перехода в RL. RL_ZERO означает работающий actor с нулевой командой скорости, а не нулевые действия моторов. В момент аварийного перехода последний принятый inference занял 8,539 мс. Следующий результат с compute_ms=6,520 и observation_age_s=0,021119 отвергнут уже после защёлкивания аварии; он не мог восстановить RL.

В версии этого запуска policy_result_stale означал отсутствие первого результата в течение 40 мс либо возраст последнего принятого observation больше 40 мс. Здесь policy_ms уже ненулевой, что согласуется с ранее принятым результатом, ставшим просроченным. Timestamp observation был равен самому старому из LowState, arm feedback и arm target. Возраст этих входов до расчёта, интервал 20 мс между inference и время следующего расчёта вместе расходовали этот бюджет. В логах arm feedback старше LowState примерно на 10–15 мс. Это возможная причина истечения TTL при быстрых расчётах; точный возраст предыдущего принятого observation в момент fault в присланных строках отсутствует, поэтому причина задержки окончательно не установлена.

По требованию оператора удалены first_hold_capture_changed и отображение hold_capture_tolerance_rad. Удаление hold-проверки не меняло watchdog; отдельное исправление policy описано ниже. Физический запуск или команды переключения/моторов при разборе не выполнялись.

Сборка/установка PASS (`runtime/build_remove_capture_gate.log`); supervisor и controller contract tests 2/2 PASS (`runtime/test_remove_capture_gate.log`). Регрессия проверяет изменение measured q всех 12 суставов на 0,03 рад после capture: HOLD_CURRENT сохраняет captured target без fault, подъём разрешён. Smoke-скрипты обновлены под отсутствие удалённого поля; ROS smoke в этой проверке не запускался. Новый executable используется только после перезапуска launch оператором; уже работающий процесс не заменялся.

## Исправление проверки ответа policy и повторная сверка подъёма

Удалено смешение возраста arm inputs и watchdog ответа policy. Теперь:

| Проверка | Откуда считается время | Порог / ошибка |
|---|---|---|
| Первый ответ RL | От входа в RL до первого принятого ответа | 40 мс / policy_result_stale |
| Обновление ответа | От принятия последнего валидного результата | 40 мс / policy_result_stale |
| Незавершённый inference | От выдачи текущего policy ticket, включая подготовку/reset и ожидание mutex | 40 мс / policy_inference_timeout |
| Данные ног для конкретного inference | От захваченного LowState, проверка до расчёта и при принятии | 40 мс / policy_observation_stale |
| Данные руки для конкретного inference | От захваченных feedback/target, проверка при принятии | arm_timeout_s=0,25 с / policy_arm_observation_stale |

Отдельные проверки актуальных LowState/remote/arm/Sport сохраняются. Результат принимает только действующий ticket текущей сессии: X/B/fault, смена команды или новая сессия инвалидируют старый job. Повторный результат не обновляет часы. При задержке executor завершившийся job старше 40 мс отвергается до обновления watchdog ответа. Три последовательных compute_ms≥20 по-прежнему вызывают policy_deadline_burst. Увеличения порога 40 мс не было; поменялся источник времени именно проверки ответа.

Статус и transition logs теперь показывают policy_result_age_s, policy_inference_age_s, policy_observation_age_s. При fault значения фиксируются перед отменой job. WARN для отвергнутого результата дополнительно показывает job_age_s и возраст захваченных arm feedback/target. До первого принятого ответа result/observation age=-1; когда job отсутствует inference age=-1.

Повторно прочитаны reference mujoco_sim.cpp и go2_rars01_unified.yaml; reference не менялся. Real подъём: q(t)=q_measured*(1-u)+q_stand*u, u=clamp(t/6,0,1), затем 4 с фиксированного удержания; dq=0, tau=0, Kp=40/Kd=1 для всех 12 ног. После первого принятого RL action — Kp=25/Kd=1. До первого ответа сохраняется фиксированное удержание 40/1. В симуляторе rate=motiontime/300 и standup_done при motiontime≥500, policy_dt=0,02: та же длительность 6+4 с и те же gains. Real интерполяция обновляется на IO 500 Гц, reference на policy 50 Гц; математическая траектория совпадает, частота дискретизации различается.

Stand pose в hardware-порядке FR/FL/RR/RL: [-0.1,0.8,-1.5, 0.1,0.8,-1.5, -0.1,0.8,-1.5, 0.1,0.8,-1.5]. Production и config/profiles/go2_rars01_commissioning.yaml совпадают с reference по gains, target, joint_names, frequency и action_scale: runtime/policy_response_stand_parity.json. Проверка совпадения программных параметров не подтверждает физическое достижение позы; ошибка позы в прежнем логе оставалась примерно 0,23 рад.

Дополнительно прослежен путь до публикации: LoadR3Profile читает fixed/rl kp/kd и переводит в hardware order; Tick в STAND_TRANSITION/HOLDING вызывает MakeLowCmd(target_,profile_.kp,profile_.kd), после принятого RL action — MakeLowCmd(target_,profile_.rl_kp,profile_.rl_kd). MakeLowCmd для индексов0..11 явно записывает m.kp=kp[i], m.kd=kd[i], dq=0 и tau=0, затем вычисляет CRC. Первоначальные нули gains остаются только у неиспользуемых индексов12..19. IO callback проверяет AllowsPacket (включая точное совпадение gains с режимом), затем публикует именно этот packet через output_->publish(*packet). Другой подмены/масштабирования gains на этом пути нет. В read_only публикация отключена. Эти проверки подтверждают содержимое сформированных пакетов и путь к publisher; текущий приём физическими моторами не измерялся.

PASS: согласованная сборка/установка (runtime/build_policy_response_consistent.log), supervisor/controller 2/2 (runtime/test_policy_response_audit.log), actor contract/reset/config 3/3 (runtime/test_actor_policy_response_audit.log), verify_r1/r2/r3 (runtime/verify_policy_response_audit.log), ROS remote read-only smoke (runtime/smoke_policy_response_audit.log и runtime/node_policy_response_audit.log). Регрессия воспроизводит arm age15 мс, первый inference8,5 мс, второй job через20 мс, IO tick27 мс: ложного fault нет. Проверены задержки100/200/1000 мс, отсутствие новых результатов, stale snapshots, X/B/session cancellation, дубликаты; все 12 stand/hold packets имеют 40/1. ROS domain223/loopback подтвердил новые поля возраста, удаление hold_capture_tolerance_rad, кнопки X/B, sent_packets=0 и отсутствие LowCmd publishers. Длительный timing benchmark не повторялся. Физические команды не отправлялись; работающий launch не заменялся.

Первый ROS smoke этой правки выявил std::bad_alloc после изменения структуры контроллера во время предыдущей сборки. Все объекты, зависящие от r3_commissioning.hpp, принудительно пересобраны; повторные controller tests и ROS smoke прошли. Первичный отказ сохранён в runtime/node_policy_response_audit_initial_failed.log и runtime/smoke_policy_response_audit_initial_failed.log. Актуальный install — результат согласованной пересборки; требуется новый запуск launch оператором, уже работающий процесс его не подхватывает.


## Изменение запуска руки 05.10.2026

По прямому требованию оператора удалена постоянная блокировка старта по journal поверх runtime watchdog. Удалены BlockReason, чтение/проверка старых FAULT/ENABLE_ATTEMPT/invalid записей и enable_once_on_boot. Journal служит диагностике; ошибка записи не запрещает enable. Новая сессия явно запускается оператором и проходит обычный connect →10 с→enable→HOME.

Runtime watchdog, свежесть feedback/targets, проверка motor ID/validity и serial leases сохранены. Fault останавливает текущую сессию без re-enable; новый запуск owner выполняет новую попытку. Systemd Restart=no, автоматического перезапуска процесса нет.

Ранее отказ был усилен ошибкой, затиравшей причину runtime_watchdog при повторном запуске. Исходный journal с согласия оператора архивирован в runtime/arm_journal_diagnostics/recovery_20261005_100731/. Теперь архивирование/очистка больше не нужны для запуска. Причина watchdog04.10 в19:20:38 не установлена. STM32 подключён на host; прежнее сообщение об отсутствии USB было неверным из-за sandbox.

Проверки PASS: сборка/установка (`runtime/build_arm_no_journal_guard.log`),3/3 targeted tests (`runtime/test_arm_no_journal_guard.log`: AUTO HOME, isolation, R3 supervisor), verify_r1/r2/r3 (`runtime/verify_arm_no_journal_guard.log`). Mock tests подтверждают новый старт при старом FAULT, same-boot ENABLE_ATTEMPT и invalid bytes, а также остановку stream без re-enable при текущем watchdog. Физический owner не запускался, enable/targets не отправлялись. Команда запуска руки из README остаётся той же; новый executable используется при новом запуске owner.


## System FSM: PHASE A 05.10.2026

Оператор сообщил об успешной работе текущего RL_ZERO после предыдущего исправления (OPERATOR_REPORTED; новые логи не приложены). По заданию CODEX_SYSTEM_FSM_REFACTOR (1).md выполнены только audit/design и [план PHASE A](CODEX_SYSTEM_FSM_REFACTOR_PLAN.md). Runtime baseline39b78c9 не изменён. PHASE B, новые профили и целевые X с lie-down/B с emergency руки не реализуются до явного approval этого плана. Bench/физические подтверждения новой траектории и emergency руки отсутствуют.

## System FSM: PHASE B approval 05.10.2026

PHASE A утверждена файлом CODEX_SYSTEM_FSM_PHASE_B_APPROVAL.md; внедрение выполнено в отдельном build_fsm/install_fsm, текущий physical install_r1 и процессы не заменяются. Новые профили и X/A/B описаны в [README](../../README.md), детали/commits/tests — в [отчёте Phase B](CODEX_SYSTEM_FSM_PHASE_B_REPORT.md). NAV clamp сохранён. Lie target approved, trajectory dynamics и arm emergency disable остаются не commissioned; соответствующие flags false. NAV/FULL требуют реальных adapters, которых пока нет. Предыдущие разделы сохраняют историю baseline, не описывают новый X.


### PHASE B — offline CPU timing после остановки launch

05.10.2026 оператор подтвердил завершение теста и остановку launch ног. Host process check не обнаружил controller ног; агент процессы и руку не останавливал. В candidate install_fsm выполнен policy_cpu_test: PASS, CPU actor315→12, threads1, по50 warmup и500 samples. JIT mean1,06489/p99 5,5221/max15,27 мс; полный agent mean1,34663/p99 1,72123/max2,43603 мс при20 мс. Вместе с26 пройденными regression tests это27/27. Лог runtime/fsm_cpu_timing.log, итоговый отчёт CODEX_SYSTEM_FSM_PHASE_B_REPORT.md. Это короткое synthetic измерение, не подтверждение длительной нагрузки LiDAR/mission/physical IO. Runtime/watchdog thresholds, dynamics и arm emergency validation gates не менялись; физических команд не отправлялось.


### PHASE B.1 — успешный X освобождает leg output

05.10.2026 реализовано по CODEX_SYSTEM_FSM_PHASE_B1_LIEDOWN_OUTPUT_OFF.md. После zero RL/HOME, measured PD handoff, smooth approved lie-down и непрерывного reached/settle: stop publishing → destroy publisher → release lease → confirmed SYSTEM_HOLD; без Sport enable и без заявления electrical power-off. Новый A из output-OFF hold проходит прежние foreign graph/file-lease gates, reacquires output и начинает с fresh measured_q; прежний stand6/hold4/reset/RL сохранён. Timeout/not-reached удерживает fixed target/output ON; B и поздние tickets/generation guards сохранены.

Trial включается явно при startup controlled_stop_lie_down_trial:=true; production и operator YAML defaults false, lie_down.operator_validated=false. Это разрешение commissioning, не physical validation. Remote_test и NAV semantics неизменны. Сборка candidate install_fsm PASS,28/28 regression PASS, initial A differential byte-identical, read-only loopback ROS smoke и verify_r1/r2/r3 PASS. Первый smoke выявил YAML ON/OFF boolean parsing: статус исправлен на quoted strings; повтор PASS. Логи runtime/fsm_stepb1_* и runtime/fsm_b1_*, отчёт CODEX_SYSTEM_FSM_PHASE_B1_REPORT.md. Изменения B.1 не закоммичены/не запушены; физических команд и остановки физических процессов агентом не было. IMU utility остаётся отдельным Sport calibration, не запускается одновременно с ACTIVE custom RL; путь ~/Desktop/imu_calib_data.yaml сохранён.


### 06.10.2026 — B.1 commit и PHASE B.2 PASSIVE

B.1 сохранён отдельным e2990c6 до начала B.2, без push. По CODEX_SYSTEM_FSM_PHASE_B2_PASSIVE_MODE.md успешный X дополнен explicit PASSIVE: mode0 для12 ног, kp/kd/tau0, прежние q/dq sentinels2.146E9/16000 и native CRC. Через существующий IO500Гц отправляются10 canonical packets; счётчик — после actual publish. Незавершённая серия ограничена100 мс; late publish учитывается, но не даёт false successful shutdown. Затем существующий publisher/lease stop acknowledgement и SYSTEM_HOLD. A reacquires, первый measured-q packet имеет mode1; stand6/hold4/reset/RL, B active damping0/3, actor/NAV/watchdogs/SDK/RARS неизменны.

Software: build candidate install_fsm PASS,30/30 regression/CPU tests PASS, baseline A byte-identical, read-only ROS loopback smoke и verify_r1/r2/r3 PASS. CPU full agent max1.36223 мс/20 мс, не full-system длительная нагрузка. Первый dropout fixture был задан после settle; исправлена fixture, runtime criteria не менялись, initial failed log сохранён. Статус passive_command_sent/packets/complete/last_mode не заявляет electrical power-off. Trial остаётся immutable startup override, production validation false. Команда диагностики в README исправлена на реально установленный r3_status.py.

Готовность hardware проверена повторно read-only: enP8p1s0 DOWN/NO-CARRIER, STM32 ttyACM0 видна, controller и arm owner не запущены. Физический тест требует подключения Ethernet, подтверждения fresh Sport/HOME и отдельного операторского trial; движения/LowCmd/serial/Sport enable/release агентом не отправлялись. Отчёт CODEX_SYSTEM_FSM_PHASE_B2_REPORT.md содержит команды и A/X цикл. B.2 пока не закоммичен/не запушен.


### 06.10.2026 — повторная сверка и сохранение B.2

При review перед commit устранён maintenance service обход output-OFF SYSTEM_HOLD: stand/RL/lie-down теперь требуют active output, не меняют FSM без reacquire через A. Добавлен regression deny HOLD/STAND/RL/LIE_DOWN/ENABLE_OUTPUT после shutdown. Initial A byte parity сохранена, согласованная сборка и17 targeted PASS, полный финальный прогон30/30 PASS, CPU agent max1.7335 мс/20 мс. B.2 сохраняется commit feat: command bounded passive transition after controlled lie-down, без push. Команды и результат review внесены в CODEX_SYSTEM_FSM_PHASE_B2_REPORT.md; physical trial ещё не выполнялся.


### 07.10.2026 — physical restart: policy_inference_timeout

Источник: /home/ruben/.ros/log/go2_r3_commissioning_12047_1791367515059.log. Первый RL_ZERO работал около80 с (1791367580.845 →1791367660.921), X прошёл HOME/PD/lie8 с/settle0.2/PASSIVE/SYSTEM_HOLD (1791367669.357). Повторный A reacquire/stand6/hold4 завершился; на первом RL job (1791367702.425) Act + extraction заняли221.556 мс, job223.317 мс. In-flight watchdog сработал при42.066 мс: EMERGENCY_DAMP0/3. Затем Sport observation age501.954 мс превысил прежний0.5 с, поэтому output остановлен/FAULT_LATCHED, sent_packets54206. Возраст LowState/arm был свежим в момент первого fault. policy_ms4.880 в transition — предыдущий завершённый inference, не длительность текущего job; результат221.556 вернулся позже и корректно отвергнут. reset/подготовка вне compute заняли лишь около1.76 мс; дорогое reset не подтверждается. compute_ms — wall time Act + extraction, включает ожидание планировщика; причина (Torch/allocator/CPU scheduling/power/прочее) не установлена. Startup прогрев существует; отсутствие прогрева не заявляется причиной. Нужна отдельная offline restart/idle диагностика и разбиение времени внутри Act; watchdogs/actor не изменены.

Обрезанное data: относится к ros2 topic echo, не к payload/FSM. Полный вывод: ros2 topic echo /go2/locomotion_status --full-length; структурированный YAML: ros2 run unitree_legged_real r3_status.py --timeout 3600. Опция --full-length подтверждена локальным Jazzy --help. На host при диагностике leg controller уже отсутствовал, arm owner PID11632 работал; его не трогали. CPU текущий schedutil1267200 kHz — snapshot после инцидента, не доказательство причины. Физических команд/изменений control code не было. Новый цикл физически прошёл X и повторный stand, но restart RL стабильным PASS пока не является.


### 07.10.2026 — runtime autograd defect и REMOTE +/-0.5

По запросу оператора remote.command_limits max_vx/max_vy/max_wz=0.5 в production/trial YAML; отдельные fields/fallback в R3Profile. Стики ly/-rx/-lx и deadband прежние, NAV/manual0.2/0.1/0.1 не менялись. Статус показывает remote/navigation limits.

Проверкой real policy установлен runtime defect: все8 parameters requires_grad=true; Node warmup/worker не имели InferenceMode, а CPU tests имели. Actual C++ Agent подтверждает grad output и (со второго frame) grad history. Previous-action/history удерживают старые graphs; освобождение старого output graph внутри Act или JIT specialization при reset могут объяснять221.556 мс, но stack profiling инцидента отсутствует. Исправлен Node startup и отдельный callback thread-local InferenceMode, охватывающий feedback/reset/history/Act/extraction; Agent/core/weights/mapping/history math/watchdogs не менялись.

Numeric parity30 frames PASS;5x200 worker restart frames no graphs, max firstAct2.45929 мс. Full32/32 PASS, CPU agent max1.54313 мс/20 мс, ROS read-only loopback smoke и verify_r1/r2/r3 PASS; initial A byte oracle сохранён. Первое утверждение fixture о grad history на первом frame исправлено (initial previous_action zero); отказ сохранён, повтор PASS. Новая физическая ходьба/restart не запускались агентом. Leg controller отсутствовал, arm owner PID11632 не трогали. Полные команды/ограничения в CODEX_REMOTE_WALK_RESTART_REPORT.md, workspace trial/README/report синхронизированы. Изменения не закоммичены/не запушены.


## 07.10.2026 — аудит складывания после X, подъёма и RL данных

Последний physical log28119: X отменяет быстрый inflight result4.95ms, следующий job8.89ms ещё выполняется, IO видит last accepted age40.615ms и вызывает policy_result_stale→emergency0/3; lie-down вообще не начался. Race воспроизведена отдельным offline C++ probe с50Гц scheduling, без publisher/SDK/serial. Runtime исправление в этом аудите не вносилось. Требуется ограниченный X→zero-policy handoff и regression при X посреди job без подделки timestamp и без снятия stalled worker watchdog. Найдено отдельное расхождение: reference manual stand пишет default[i] в hardware order, real корректно remaps default по именам; hip знаки противоположны, thigh/calf одинаковы. Agent/observation cpp побайтово идентичны reference, actor YAML равен, mapping совпадает; причин jerky walking и rear body contact без target/feedback/IMU записи доказать нельзя. Полный контракт и выводы: CODEX_RL_DATA_AND_MOTION_AUDIT.md; runtime/x_walk_pose_audit_reproduction.log. Изменена только документация.


## 07.10.2026 — исправление гонки X и commit накопленных изменений

В ACTIVE X теперь проверяет исходный job/result deadline, отменяет pre-X ticket, ограниченно40мс удерживает последний использованный target при прежних gains до первого принятого zero-command result. Отдельный handoff timestamp не подменяет policy freshness; repeated X его не продлевает. Timeout/late completion→policy_zero_handoff_timeout; B/invalid inputs/expired worker сохраняются. Добавлен system_x_policy_handoff_test с20мс worker/2мс IO и status policy_zero_handoff_pending. Полный suite32/33: все функциональные тесты PASS, policy_cpu_test FAIL (final JITmax13.2325мс, Agentmax24.9181мс). Release build/verify/read-only ROS smoke PASS, физические команды не отправлялись. Подробности и гипотезы jerky walking — CODEX_X_HANDOFF_FIX_AND_RL_JERK_REPORT.md. В reference активен ROS bridge500Гц с PD recompute и torque clamp; SDK2 bridge disabled, объяснение через stale simulated torque неприменимо. Pose/gains/actions не фильтруются и не меняются по догадке. Коммитятся также ранее незакоммиченные REMOTE±0.5, InferenceMode и отчёты; push выполняет оператор.


## 07.10.2026 — временная диагностика движения, gains без изменения

Добавлен выключенный по умолчанию CSV motion logger: startup flags enabled/duration120/pathprefix, fixed queue256/try_lock, writer thread, auto-stop от первого sample и отдельный Trigger stop_motion_log. IO sample~50Гц с min/max каждого фактического publish500Гц; policy sample на каждом Act с command/sticks/clipped action/target/accepted/compute/job/captured ages. Статус active/dropped. Stop не меняет режимы/команды; после stop нет новых диагностических сборов, очередь ограниченно дописывается. Включённая диагностика имеет ненулевую нагрузку; физическая запись агентом не выполнялась. Functional33/33 (CPU benchmark excluded), build/verify/read-only logger service smoke PASS; прежний timing FAIL не закрыт. Gains не менялись, current runtime YAML rl_kp/rl_kd=25/1, fixed_kp/fixed_kd=40/1 (stand/hold/lie совместно). Подробности: CODEX_TEMP_MOTION_LOGGING_AND_GAINS.md. Новые изменения не закоммичены.


## 07.10.2026 — нормализация структуры конфигураций и документов

Операторский YAML перенесён без изменения содержимого в src/unitree_ros2_to_real/config/profiles/go2_rars01_commissioning.yaml; SHA73c9d8f09e68818478bd43cdcb5739f79bfc794cc77d268b98cb63e2ada70ee5. Удалены прежние runtime/deployment-workspace_runtime копии и generated stale install_fsm копия. Source/installed profile байт-в-байт равны. Активные README/отчёты проекта обновлены на новый config_path. Корневой workspace README стал индексом; старые корневые задания и report snapshots перенесены в docs/archive, исторические probes — docs/archive/probes; ничего из них не запускалось. Smoke node output направлен в runtime в working/versioned helper. Реальная архитектура и новый путь: PROJECT_STRUCTURE.md. Build/install,2 launch/lifecycle tests,verify_r1/r2/r3 PASS. Gains/gates/pose/policy/процессы неизменны. Перенос не закрывает ранее известный CPU timing вопрос. Новые изменения не закоммичены.


## 07.10.2026 — последний physical test: два успешных A/X цикла, RL20/1,1

Разобран go2_r3_commissioning_46959_1791378074700.log (201 transition, завершён SIGINT). Startup подтверждает source config/profiles/go2_rars01_commissioning.yaml, RL20/1,1 и fixed40/1. Настройка оператора сохранена. Два подъёма6с/hold4с/RL и два X→HOME→PD→lie8с→verify→PASSIVE→outputOFF прошли без WARN/ERROR/faults, включая restart. B в файле отсутствует. Transition samples: policy_ms median4,00/max8,36мс, accepted result age max19,21мс; это не полный timing benchmark. FL calf index5 при входе в RL отклоняется от fixed stand target на0,230/0,289рад. В walking stand_error не является ошибкой tracking policy target. Подробной motion CSV нет; причина оставшейся резкости не установлена. По коду command без ramp и deadband0,01, при любом changed command инвалидируется pending generation; влияние на rejected inference проверить CSV, не объявлять доказанной причиной. Следующий operator test — текущие20/1,1 плюс logger120с. Код/gains/gates/процессы не менялись. Отчёт CODEX_LATEST_PHYSICAL_LOG_ANALYSIS.md; README и таблица gains актуализированы.


## 07.10.2026 — CSV прогон и автоматический fault

Log71584 + go2_motion_trace-12968479839671.csv: RL20/1,1, fixed40/1. В1791379924.236 fault policy_result_stale: last accepted age41.858мс, pending22.377мс; затем job compute22.488/job22.922мс отвергнут после fault. В1791379924.391 Sport-age500.850мс→Released=false→outputOFF/FAULT_LATCHED, ещё64 публикации между transitions; подтверждения повторного Sport enable нет. CSV120с завершился~10.5с доfault:11175rows/5495policy allaccepted, compute max18.757мс. IO targets скачут до1.547рад/20.31мс; в RL_ACTIVE119sample jumps>.5рад,66при unchanged command; policy rows подтверждают большие изменения и при неизменном vx. Это не фактический мгновенный поворот сустава. Причина actor spikes не установлена; code/gains/timeouts не менялись. Отчёт CODEX_CSV_AUTO_SHUTDOWN_ANALYSIS.md, statistics runtime/latest_auto_shutdown_analysis/csv_statistics.json. Для записи конца следующего operator run предложено240с; запуск агентом не выполнялся.


## 07.10.2026 — единый README и сохранение изменений

Запуск руки/ног, пульт, CSV240с, gains20/1,1 и текущие ограничения собраны в корневом README репозитория workhop_rl. Удалены workspace индекс и дополнительные README docs/sim2real, config и deployment; RL data audit переименован в CODEX_RL_DATA_AND_MOTION_AUDIT.md, содержимое сохранено. Markdown ссылки и архитектура обновлены; upstream SDK/симулятор и reference не изменялись. Сохраняются накопленные CSV logger, source profile и отчёты, без исправления подтверждённых policy target spikes/40мс fault. Push выполняет оператор.
