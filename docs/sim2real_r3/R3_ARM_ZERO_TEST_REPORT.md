# Реальный тест руки: первый USB frame и преждевременный fault owner

04.10.2026. По разрешению оператора выполнялся один ограниченный тест RARS с единственными позиционными целями `[0,0,0,0,0,0,0]`. LowCmd ног и Sport switching не запускались.

## Что произошло на железе

```text
serial connect
→ 10 с
→ SDK enable вернул успех
→ первый SDK USB frame без валидных motor IDs
→ наш owner немедленно FAULT_LATCHED
→ остановка процесса, existing SDK best-effort disable
```

Owner пришёл в HOLD_HOME в 15:51:00.949, затем fault в 15:51:00.960 — примерно через 11 мс. Enable был вызван один раз. `enabled_local=true` подтверждает успешный SDK control path, но не является подтверждением enabled каждого мотора.

Последний snapshot: protocol_v2_detected=true; connected=true; IDs всех семи моторов=0, valid7=false; watchdog flags=false; target_age=-1, target_valid=false, watchdog_armed=false. Accepted target отсутствовал: **тест остановился до первого успешного sendPositionTargets**. Поэтому физическое удержание нулей пока не подтверждено. Не было позиционных целей с другими углами.

Поле q=-12,5 при valid=false — результат декодирования нулевых/невалидных motor slots через protocol limits. Его нельзя интерпретировать как реальные измеренные углы или как отправленную позиционную команду.

Sport SDK --status был ACTIVE до и после теста. Существующий native DDS LowCmd endpoint остался прежним; наш тест его publisher не создавал. Процесс owner завершён, returncode=0. Повторный enable не выполнялся. Persistent FAULT journal сохранён и не удалялся.

## Сборка проверена

Запускался `/home/ruben/go2_diploma/sim2real/install_r1/unitree_legged_real/lib/unitree_legged_real/rars_r3_owner`.

До данного исправления:

- source FSM: 15:37:53;
- compiled library: 15:38:44;
- build/install executable: 15:38:44;
- Build ID обоих binaries: `b3d2d574766e252e733ecb9b9b363827eef3187e`.

Разные file SHA build/install не означали устаревший код: установка меняет ELF runtime paths, Build ID одинаков. Предыдущая сборка завершилась успешно (5 packages). **Необновлённая сборка не была причиной этого fault.** После нового исправления выполнена ещё одна сборка.

## Точная причина в нашей FSM

`rars_arm_sdk/src/arm_motor_control.cpp:126` возвращает true, когда получен USB payload. Каждый motor decode отдельно устанавливает valid=false, если ID не совпадает. Поэтому USB frame может быть получен раньше полноценного семимоторного CAN feedback.

Наш initial grace проверял `last_read_ отсутствует`. При первом неполном USB payload last_read_ уже присутствовал, и grace преждевременно прекращался. Owner прерывал запуск до первого HOME send. Это ошибка нашей интеграции; данных о неисправности STM или всех семи моторов такой snapshot не даёт.

## Исправление offline

Grace теперь действует **до первого полного usable motor feedback**, а не до первого произвольного USB frame. Неполные startup payloads с отсутствующими IDs допускают только ограниченное ожидание с zero HOME streaming; readiness=false.

- Real motor fault, неверный nonzero ID и неконечные данные valid motor по-прежнему немедленно блокируются.
- После первого полного motor feedback исчезновение/порча данных не маскируется initial grace.
- Если полный feedback не появился до grace deadline — fault и stop sends, без повторного enable.
- Targets строго семь нулей; existing SDK, calibration, direction/offsets не изменены.

Добавлен mock regression с тем же первым кадром: IDs=0, valid=false, decode q=-12,5 → отправка zero HOME без readiness → частичный frame → полный frame → readiness. Отдельно проверены never-complete timeout, исчезновение мотора после readiness и явные ошибки во время startup.

Физический тест с новой обработкой кадров **ещё не повторялся**. Journal восстановления не обходился и не стирался.

## Артефакты

- `runtime/arm_zero_test/owner.log` — реальные переходы FSM.
- `runtime/arm_zero_test/summary.yaml` — итог физического теста.
- `runtime/arm_zero_test/result.json` — все наблюдённые status samples.
- `runtime/rars_auto_home/enable-journal` — сохранённый fault.
- `build_r3_initial_frame_fix.log` и `test_r3_initial_frame_fix.log` — новая сборка/regression.

Новая сборка: 5 packages PASS, 36,3 с. Полный regression: 17 tests, 0 errors/failures/skipped; frame/history parity 8e-7. SDK working tree clean.
