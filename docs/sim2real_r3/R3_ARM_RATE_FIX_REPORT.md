# RARS01: частота HOME stream — причина, исправление и физическая проверка

04.10.2026. Только seven-zero HOME; LowCmd ног и Sport switching не запускались.

**Причина снижения частоты найдена и исправлена. В повторном физическом тесте измерено 100.015 Гц успешных SDK отправок; HOME readiness сохранялась 10.007 с.**

## Почему раньше было около 84 Гц

По прежним сохранённым owner status: callbacks/публикации шли около 100,002 Гц, но 165 из 1005 ready status сообщений содержали повторный accepted timestamp. Это не объясняется одним лишь пропуском сообщений subscriber.

Owner имел два планировщика: ROS wall timer 10 мс и внутренний `now >= next_send_`. При обычном джиттере callback мог прийти чуть раньше независимой сетки внутреннего дедлайна: такой callback не отправлял команду. Следующий callback приходил ещё через 10 мс. Из-за этого actual send rate снижался.

## Что изменено

- ROS timer является единственным расписанием отправок owner. Каждая его итерация после enable выполняет один send при соблюдении readiness/fault условий.
- В core добавлен explicit command_timer_tick: только настоящий configured-rate callback обходится без второго rate limiter. Другие callers сохраняют внутренний limiter.
- Нет catch-up burst после пропущенных callbacks; faults/watchdog/stream stale gates сохраняются.
- Добавлены successful_sends, first_accepted_ns, last_send_gap_ms и max_send_gap_ms. Счётчик растёт только после успешного SDK Send; rejected send его не обновляет.
- Targets неизменно `[0,0,0,0,0,0,0]`; SDK gains, directions, calibration и offsets не менялись.

## Offline-проверки

Сборка: 5 packages PASS, 36,8 с. Полный suite: **17 tests, 0 errors/failures/skipped**, observation parity 8e-7.

Новый regression: 100 callback ticks с чередующимся ±0,2 мс jitter → ровно 100 successful sends, один enable. Send failure не увеличивает counter. Delayed callback отправляет один target без догоняющей пачки. Старые internal-rate 50/100 Гц, initial incomplete frames, watchdog и journal tests прошли.

## Реальный повторный тест

Оператор разрешил только нулевые цели и проверку частоты. Serial был свободен. Journal предыдущего успешного теста архивирован отдельно для одной разрешённой recovery; повторного автоматического enable loop нет.

```text
connect → 10 с → SDK enable один раз
→ zero HOME stream → valid seven-motor feedback
→ readiness около 10 с → завершение owner
```

| Метрика | Результат |
|---|---:|
| Successful SDK sends | 1010 |
| Частота counter за весь stream | 100.015 Гц |
| Частота counter в ready window | 100.001 Гц |
| Максимальный межотправочный gap | 11.528 мс |
| Максимальный feedback age в ready window | 13.000 мс |
| Максимальная абсолютная HOME error | 0.005150 рад |
| Непрерывная наблюдённая readiness | 10.007 с |

Все семь IDs=1..7, motor status=1, valid=true. `arm_home_ready=true`, SDK/STM watchdog=false, last_error пуст. Observer не выявил ненулевых HOME или accepted targets. Возвращённые measured q/dq могут быть ненулевыми — это реальные измерения, не позиционные команды.

Частота посчитана по counter delta и соответствующим accepted timestamps; потеря status сообщения не уменьшает counter delta. Это измерение успешных вызовов SDK serial send, не отдельный анализатор CAN wire/individual motor feedback frequency.

## Завершение и ограничения

Sport status до/после: ACTIVE. Наш LowCmd publisher не создавался; существующий bare DDS endpoint не менялся. Owner завершён SIGINT штатно, returncode=0; SDK destructor выполняет best-effort disable. Журнал нового enable attempt сохранён. Постоянный systemd owner не запущен.

**Пункт снижения частоты до 84 Гц закрыт в этом коротком физическом тесте.** Не заявляется hard real-time гарантия или проверка под полной нагрузкой perception/GraspNet/навигации. Individual CAN freshness, fault/watchdog physical scenarios и leg commissioning остаются отдельными проверками.

## Файлы и артефакты

Изменены include/rars_auto_home.hpp, src/rars_auto_home.cpp, src/rars_r3_owner.cpp и tests/rars_auto_home_test.cpp пакета unitree_legged_real.

Workspace логи: build_r3_arm_rate_fix.log, test_r3_arm_rate_fix.log. Физические данные: runtime/arm_rate_test/owner.log, summary.yaml, result.json и metrics.json. Предыдущий successful test/journal архивированы в runtime/arm_zero_test/success_before_rate_fix.
