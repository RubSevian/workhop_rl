# Sim2Real Go2 + RARS01: актуальные решения

Обновлено: 05.10.2026. Порядок запуска: [README.md](README.md).

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

Production config сохраняет неподтверждённые допуски закрытыми. `runtime/r3_first_rl_zero.yaml` — отдельно принятый оператором экспериментальный профиль; его флаги не являются измеренным PASS.

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

Stand pose в hardware-порядке FR/FL/RR/RL: [-0.1,0.8,-1.5, 0.1,0.8,-1.5, -0.1,0.8,-1.5, 0.1,0.8,-1.5]. Production и runtime/r3_first_rl_zero.yaml совпадают с reference по gains, target, joint_names, frequency и action_scale: runtime/policy_response_stand_parity.json. Проверка совпадения программных параметров не подтверждает физическое достижение позы; ошибка позы в прежнем логе оставалась примерно 0,23 рад.

Дополнительно прослежен путь до публикации: LoadR3Profile читает fixed/rl kp/kd и переводит в hardware order; Tick в STAND_TRANSITION/HOLDING вызывает MakeLowCmd(target_,profile_.kp,profile_.kd), после принятого RL action — MakeLowCmd(target_,profile_.rl_kp,profile_.rl_kd). MakeLowCmd для индексов0..11 явно записывает m.kp=kp[i], m.kd=kd[i], dq=0 и tau=0, затем вычисляет CRC. Первоначальные нули gains остаются только у неиспользуемых индексов12..19. IO callback проверяет AllowsPacket (включая точное совпадение gains с режимом), затем публикует именно этот packet через output_->publish(*packet). Другой подмены/масштабирования gains на этом пути нет. В read_only публикация отключена. Эти проверки подтверждают содержимое сформированных пакетов и путь к publisher; текущий приём физическими моторами не измерялся.

PASS: согласованная сборка/установка (runtime/build_policy_response_consistent.log), supervisor/controller 2/2 (runtime/test_policy_response_audit.log), actor contract/reset/config 3/3 (runtime/test_actor_policy_response_audit.log), verify_r1/r2/r3 (runtime/verify_policy_response_audit.log), ROS remote read-only smoke (runtime/smoke_policy_response_audit.log и runtime/node_policy_response_audit.log). Регрессия воспроизводит arm age15 мс, первый inference8,5 мс, второй job через20 мс, IO tick27 мс: ложного fault нет. Проверены задержки100/200/1000 мс, отсутствие новых результатов, stale snapshots, X/B/session cancellation, дубликаты; все 12 stand/hold packets имеют 40/1. ROS domain223/loopback подтвердил новые поля возраста, удаление hold_capture_tolerance_rad, кнопки X/B, sent_packets=0 и отсутствие LowCmd publishers. Длительный timing benchmark не повторялся. Физические команды не отправлялись; работающий launch не заменялся.

Первый ROS smoke этой правки выявил std::bad_alloc после изменения структуры контроллера во время предыдущей сборки. Все объекты, зависящие от r3_commissioning.hpp, принудительно пересобраны; повторные controller tests и ROS smoke прошли. Первичный отказ сохранён в runtime/node_policy_response_audit_initial_failed.log и runtime/smoke_policy_response_audit_initial_failed.log. Актуальный install — результат согласованной пересборки; требуется новый запуск launch оператором, уже работающий процесс его не подхватывает.


## Изменение запуска руки 05.10.2026

По прямому требованию оператора удалена постоянная блокировка старта по journal поверх runtime watchdog. Удалены BlockReason, чтение/проверка старых FAULT/ENABLE_ATTEMPT/invalid записей и enable_once_on_boot. Journal служит диагностике; ошибка записи не запрещает enable. Новая сессия явно запускается оператором и проходит обычный connect →10 с→enable→HOME.

Runtime watchdog, свежесть feedback/targets, проверка motor ID/validity и serial leases сохранены. Fault останавливает текущую сессию без re-enable; новый запуск owner выполняет новую попытку. Systemd Restart=no, автоматического перезапуска процесса нет.

Ранее отказ был усилен ошибкой, затиравшей причину runtime_watchdog при повторном запуске. Исходный journal с согласия оператора архивирован в runtime/arm_journal_diagnostics/recovery_20261005_100731/. Теперь архивирование/очистка больше не нужны для запуска. Причина watchdog04.10 в19:20:38 не установлена. STM32 подключён на host; прежнее сообщение об отсутствии USB было неверным из-за sandbox.

Проверки PASS: сборка/установка (`runtime/build_arm_no_journal_guard.log`),3/3 targeted tests (`runtime/test_arm_no_journal_guard.log`: AUTO HOME, isolation, R3 supervisor), verify_r1/r2/r3 (`runtime/verify_arm_no_journal_guard.log`). Mock tests подтверждают новый старт при старом FAULT, same-boot ENABLE_ATTEMPT и invalid bytes, а также остановку stream без re-enable при текущем watchdog. Физический owner не запускался, enable/targets не отправлялись. Команда запуска руки из README остаётся той же; новый executable используется при новом запуске owner.
