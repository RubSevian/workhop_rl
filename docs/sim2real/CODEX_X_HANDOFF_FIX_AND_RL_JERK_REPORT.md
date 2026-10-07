# Исправление X и анализ резких движений RL

07.10.2026. Репозиторий `workhop_rl`, ветка `ros2_go2_rars01_real`. Агент не запускал физический controller, не отправлял LowCmd/Sport/STM32 команды и не останавливал руку. Перед проверками read-only список процессов подтвердил отсутствие controller ног; arm owner PID11632 оставлен работающим.

## Что исправлено

**X больше не должен вызывать ложный emergency из-за отмены быстрого inference.** До первого принятого результата с нулевой командой введён отдельный ограниченный переход. Проблема из последнего физического лога: X отменял inflight ticket, его быстрый ответ отвергался, следующий 50Гц вызов ещё выполнялся, а IO500Гц уже считал предыдущий принятый ответ старше40мс. Робот складывался потому, что контроллер включал damping0/3, не приступив к lie-down.

Новое поведение:

1. X из ACTIVE проверяет, что worker и последний результат не были уже просрочены. Уже имеющийся timeout не маскируется.
2. Ticket предыдущей команды отменяется по generation; velocity command обнуляется. History actor не сбрасывается этим переходом.
3. До нового zero-command ответа повторяется **последний реально использованный motor target** с прежними RL25/1. Если X попал до самого первого принятого actor result, сохраняется прежний fixed capture target40/1.
4. Ожидание ограничено **40мс от X**, отдельно от реального возраста предыдущего policy result. Его timestamp и observation age не обновляются искусственно. Повторный X не продлевает срок.
5. Принятый свежий zero-command результат закрывает handoff; обычный40мс result watchdog снова действует. Далее существующие HOME→measured PD→lie8с→settle→PASSIVE10→output OFF.
6. Отсутствие результата вызывает `policy_zero_handoff_timeout`. Поздний ответ не может отменить timeout, даже если IO callback задержался. Реально просроченный job, input/ownership fault и B сохраняют аварийное поведение.

Добавлено поле `policy_zero_handoff_pending` в transition trace и `/go2/locomotion_status`. Возможен корректный WARN от отменённого pre-X ticket с пустым fault: такой результат действительно нельзя принимать. Это не означает, что новый zero-command ответ отвергнут.

**Gains, 6с подъём/4с hold, 8с lie-down, actor mapping/scales/history, веса и B0/3 не менялись.** Не добавлены фильтры actions или изменение pose без подтверждения. Сохранены ранее запрошенные REMOTE±0,5 и inference-mode fix; они тоже входят в commit, потому что были незакоммичены.

## Почему ходьба может быть резкой при нормальном Sim2Sim

### Подтверждённое по коду

- Математика actor/observation идентична reference:63 числа, history5,315→12, action_scale0,25, output mapping FR/FL/RR/RL корректен. Нет подтверждённой ошибки индексов RL, масштаба градусов/радиан или quaternion order.
- Policy target меняется ступенями50Гц, а IO500Гц повторяет его без interpolation. Command от стиков тоже не имеет ограничителя ускорения. Дёрнуть стик значит изменить velocity command за один policy frame.
- Диапазон команды вырос относительно старого REMOTE: вперёд0,2→0,5 (×2,5), lateral0,1→0,5 (×5), yaw0,1→0,5 (×5). Это заметное изменение excitation при том же ходе стика. Одновременные vx=vy=0,5 дают длину planar command≈0,707м/с: покомпонентный clamp не ограничивает норму вектора до0,5.
- Нулевой command не означает нулевой action. Ненулевая corrective action может появиться даже без движения стиками, если measured pose/IMU/arm state отличаются от состояния в training.
- kp25 даёт скачок пропорциональной части момента5Н·м при Δq_des0,2рад. Если previous target требовал отклонения для поддержки веса, handoff к measured q с fixed40/1 также мгновенно меняет error/torque, хотя первый q target геометрически непрерывен относительно measured pose.
- Перед последним RL measured stand отличался от target на0,282рад по FL calf. Stand завершается по времени, не по подтверждению body pitch и tracking. Принимаемый первый policy action и gains переключаются без blending.

### Существенное уточнение про настоящий simulation bridge

Дочитана активная ветка reference, а не только SDK2 пример. `simulate/config_go2_rars01.yaml`: **enable_ros_bridge=1, enable_unitree_bridge=0**. В `simulate/src/main.cc` ROS bridge `MujocoRosLowLevelBridge::Apply` пересчитывает PD по актуальным q/dq **перед каждым physics step500Гц**, явно clamp torque±23,7 для hip/thigh и±35,55 для calf. Следовательно, объяснять нормальный Sim2Sim тем, что там PD пересчитывается лишь50Гц и держится старый torque, **неверно**. Это уточнение внесено и в предыдущий аудит.

На физическом пути мы отправляем q/dq_des/kp/kd/tau, регулятор выполняется прошивкой моторов. Сам файл YAML torque_limits не накладывает software clamp на полный физический PD torque. Точная firmware servo frequency, saturation, current limits и friction/delay здесь не измерены. Совпадение kp/kd с simulator не означает одинаковую динамику, но конкретную виновную величину по существующему логу выбрать нельзя.

### Наиболее полезные рабочие гипотезы

| Приоритет проверки | Возможный механизм | Чем отличить |
| --- | --- | --- |
| 1 | Ступеньки command и заметно увеличенный gain от стика к скорости; малый deadband0,01 | Сопоставить сырые lx/rx/ly, выбранную command и Δaction/Δq_des. Если target дёргается после command скачка, начать с command acceleration limiting, не менять policy output |
| 2 | Плохой tracking или наклон/контакт корпуса ещё до RL | Сравнить measured q/dq с target на stand/hold и roll/pitch. Если target плавный, а measured q рывками или корпус опирается на заднюю часть, сглаживание actor не устраняет причину |
| 3 | Отличие физической PD/моментной динамики, контакта, нагрузки руки от модели | При одинаковом target/input сравнить tau_est, dq, body tilt и saturation; в simulation уже есть явный torque clamp |
| 4 | Command/data задержки в реальном времени | Записать timestamps LowState/arm frame, начало/конец job, принятие result и фактический publish. Симулятор использует последовательные physics ticks, real sensors/ROS/worker живут независимо |
| 5 | Разница между SDK нулём руки и модельной нулевой позой, training state/checkpoint | Сверить реальные joint axes/zero/pose и массу/монтаж с URDF; manifest checkpoint/commit пока null. Нулевые SDK targets сами по себе не доказывают одинаковые физических положения и COM |

Это гипотезы, не установленные причины. Последний trace не содержит raw action12, target12, measured q/dq12 и body orientation во времени; утверждение «виноват kp» или «нужно снизить kd» по нему было бы догадкой.

## Подъём: что поправлено, а что оставлено

Ошибка падения после X исправлена в FSM. Позу подъёма не разворачивал вслепую. Найденное в предыдущем отчёте отличие действительно существует: reference manual MODE_STANDUP пишет default[i] в hardware FR/FL/RR/RL, без mapping; real использует именованную actor pose. Поэтому reference manual hip signs [+0,1,−0,1,+0,1,−0,1], real [−0,1,+0,1,−0,1,+0,1]. Thigh0,8/calf−1,5 одинаковы.

Текущий real соответствует joint names actor, а reference RL уже remaps action правильно. Копирование manual stand bug в actor observation испортило бы контракт. Чтобы объяснить именно упор задней частью, нужны body pitch и rear q tracking на подъёме; имеющийся worst error FL calf не доказывает ошибку индексов задних ног. Это не объявлено исправленной физической позой.

## Проверки

- Финальная Release/Jazzy/aarch64 сборка candidate install_fsm — PASS (1мин52с).
- Новый X handoff regression — PASS, включая late completion после40мс и already expired result/job.
- Полный CTest: **32/33 PASS**, единственный FAIL — `policy_cpu_test`. Все функциональные FSM/packet/actor-contract/reset/numeric-parity tests прошли.
- Первый timing-прогон: JIT max25,6163мс. Финальный после последних изменений: JIT max13,2325мс, полный Agent max24,9181мс при period20мс. Это wall-clock offline measurements, не доказательство конкретной причины дёрганья на роботе. Порог теста не снижался и результат не скрыт; timing остаётся незакрытым.
- Одно превышение20мс само по себе не равно немедленному emergency: runtime отдельно проверяет job/result freshness40мс и три последовательных compute misses≥20мс. Эти механизмы сохранены.
- verify_r1/r2/r3 — PASS (один verify_r3 последовательно вызывает остальные).
- Собранный node: ROS read-only localhost/domain223 smoke — PASS; status, immutable profile, synthetic X/B, NAV clamp/freshness, zero LowCmd publishers. Рука/SDK/физическая сеть в smoke не использовались.
- git diff --check — PASS.

Логи: `runtime/x_handoff_fix_final_build.log`, `runtime/x_handoff_fix_regression.log` (первый прогон), `runtime/x_handoff_fix_final_regression.log`, `runtime/x_handoff_fix_verify.log`, `runtime/x_handoff_fix_ros_smoke.log`. Не заявляется all-tests-green или физически проверенная плавность ходьбы.

Новый `system_x_policy_handoff_test` моделирует 50Гц worker и2мс IO, а не принимает новый action на каждом IO tick. Он проверяет X при разных фазах старого ответа/job, 5мс отменённый ответ, 9мс новый zero ответ, побайтовое удержание packet, сохранение честного возраста, repeated X, отсутствие ответа, late completion при задержанном IO, B, arm fault и уже просроченный worker/result. Существующие controlled-stop tests проверяют полный HOME/PD/lie/PASSIVE/OFF цикл. Физическая проверка исправленного X агентом не выполнялась.

## Файлы и commit

Изменения этой задачи: r3_commissioning.hpp/cpp, go2_r3_commissioning.cpp, system_x_policy_handoff_test.cpp, system_controlled_stop_test.cpp, CMakeLists.txt и документация. Также коммитятся накопленные REMOTE±0,5, runtime InferenceMode, их tests и предыдущие отчёты. Reference/SDK/arm targets/weights не изменялись.

Commit message: `fix: bound X policy handoff and stabilize remote inference`.

Push выполняет оператор, агент не пушил:

```bash
git -C /home/ruben/go2_diploma/sim2real/repos/workhop_rl push -u origin ros2_go2_rars01_real
```

ID и содержимое commit:

```bash
git -C /home/ruben/go2_diploma/sim2real/repos/workhop_rl log -1 --oneline
git -C /home/ruben/go2_diploma/sim2real/repos/workhop_rl show --stat HEAD
```

После сборки нужен новый leg launch из candidate install_fsm: уже работающий процесс автоматически не получает изменение. Руку заново запускать ради этого не требуется. Результаты offline не заменяют проверку реальной динамики X/подъёма/ходьбы. Команды запуска и данные315-D остаются в README и [аудите контракта](CODEX_RL_DATA_AND_MOTION_AUDIT.md).
