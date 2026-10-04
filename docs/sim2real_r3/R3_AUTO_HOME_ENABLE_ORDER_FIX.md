# Исправление AUTO HOME: enable раньше motor feedback

04.10.2026. Уточнение оператора: STM выдаёт motor feedback после enable. Изменён только интеграционный owner; SDK не изменялся. На железе enable/targets/LowCmd не запускались.

**Ошибка была в нашей FSM:** она требовала семимоторный feedback до enable, а затем ждала feedback до первого HOME send. Это не соответствует согласованному startup и примеру SDK.

## Сверка SDK

- `rars_arm_sdk/src/rars_arm.cpp:83`: enable посылает enable frames, сбрасывает receiver statistics и оставляет feedback watchdog unarmed.
- `src/rars_arm.cpp:196`: sendWithModes требует открытый receiver и локальный enabled. Первый send не требует предыдущего feedback при unarmed watchdog.
- `src/rars_arm.cpp:305`: успешная отправка вооружает feedback watchdog.
- `src/rars_arm.cpp:539`: пока первого feedback нет, watchdog использует configured initial_feedback_grace.
- `examples/arm_sdk_test.cpp:313`: GUI вызывает enable и затем запускает send_timer. Получение feedback не является условием запуска timer.
- В исходном CODEX_R3_RARS01_AUTO_HOME.md пример также задаёт enable → sendPositionTargets → tryReadJointState.

SDK подтверждает допустимость этой последовательности. Отсутствие pre-enable feedback конкретной STM подтверждено оператором; read-only тест показал ожидаемое отсутствие кадров и не доказал неисправность STM.

## Исправленный порядок

```text
connect: serial открыт, SDK receiver работает
→ 10 с после наблюдения connected
→ durable enable-attempt journal
→ SDK enable один раз
→ первый и последующие HOME send на timer, не ожидая feedback
→ настоящий feedback всех 7 моторов
→ arm_home_ready, только если соблюдены все health/freshness/HOME условия
```

До enable feedback не блокирует countdown. Потеря connection сбрасывает countdown. Известный watchdog trip по-прежнему блокирует включение.

После enable отсутствие **первого** feedback допускается только в initial grace; HOME stream в этом окне идёт, readiness остаётся false. Grace берётся из SDK configuration (в existing deployment config 1500 мс); owner консервативно отсчитывает её от начала enable call, включая его protocol delay. Если первый sample не пришёл вовремя — FAULT_LATCHED, stop sends, никакого повторного enable.

Полученный, но плохой/stale motor frame не маскируется initial grace. Вооружённые SDK/STM watchdog, disabled после grace, send failure и stream timeout также фиксируют fault. Измерения никогда не подменяются нулями. Accepted target timestamp обновляется только после successful send; HOME readiness не подменяется target readiness.

HOME остаётся `[0,0,0,0,0,0,0]`. Leg takeover лишь читает readiness; LowCmd и Sport switching не разрешались этой коррекцией. Read-only defaults и persistent fault/re-enable protection сохранены.

## Проверки

- Mock: нет pre-enable feedback → до 10 с enable=0 → после задержки enable=1 → HOME sends идут без первого feedback → feedback появляется → readiness=true.
- Mock: feedback так и не появился → после grace fault, дальнейшие sends прекращены, enable остаётся 1.
- Existing bad-ID/NaN/disabled/watchdog/stale/send-failure/reconnect/journal и leg readiness regressions сохранены.
- Повторная сборка: 5 packages PASS, 17,0 с.
- Общий offline suite: **17 tests, 0 errors, 0 failures, 0 skipped**. Observation parity `8e-7`, PASS.
- SDK working tree clean; SHA f90278b46125f2b311e4555173321e80a6c7be3f.

Изменены `src/rars_auto_home.cpp`, `tests/rars_auto_home_test.cpp` и документация. Логи: `build_r3_enable_order_fix.log`, `test_r3_enable_order_fix.log`; тестовый лог сохранён также в repository docs.

**Физическое подтверждение enable → HOME → feedback ещё не проводилось.** Read-only отсутствие feedback теперь трактуется корректно. Наличие чужого DDS LowCmd endpoint — отдельный пункт, этим исправлением ownership guard не менялся.
