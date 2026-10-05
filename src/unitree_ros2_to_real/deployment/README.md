# Deployment RARS01 AUTO HOME

[Запуск в workspace](../../../../../README.md). [Версионная инструкция в репозитории](../../../docs/sim2real/README.md).

## Файлы

| Файл | Назначение |
|---|---|
| `rars01-owner.service` | Подготовленная systemd-служба одного serial owner |
| `rars01-owner.env.example` | Пути workspace, сохранённой SDK calibration, serial device, leases и journal |
| `run_rars01_owner.sh` | Запуск owner из установленного Jazzy workspace |
| `workspace_scripts/` | Versioned snapshot помощников `sim2real/scripts/` |
| `workspace_setup.bash` | Копия `sim2real/setup.bash`; перед использованием копируется в корень workspace |
| `workspace_runtime/r3_first_rl_zero.yaml` | Копия текущего экспериментального профиля, используемого оператором |

## Поведение службы

После соединения с STM32: задержка 10 с, один enable, HOME семи моторов на 100 Гц. Служба не запускает LowCmd и не переключает Sport. `Restart=no`; journal — диагностический лог enable attempt/fault, его содержимое не блокирует следующий запуск. В текущем процессе после fault нет повторного enable. Новая сессия начинается только новым запуском owner, снова с задержкой10 с.

Служба подготовлена для отдельной установки; обычный README запускает owner вручную. Не запускайте service и manual owner одновременно. GUI/direct serial клиенты должны соблюдать то же владение устройством.

SDK destructor при намеренном завершении owner делает best-effort disable. Завершение controller ног не завершает arm owner.

Snapshot `workspace_scripts/` копируется в `sim2real/scripts/`: его относительные пути рассчитаны на workspace, не на выполнение из этой папки.

`workspace_runtime/r3_first_rl_zero.yaml` сохраняет ранее принятые оператором экспериментальные флаги. Это не production default и не подтверждение физических проверок. Для восстановления прежнего ручного запуска файл копируется в `sim2real/runtime/r3_first_rl_zero.yaml`; копирование само по себе ничего не запускает.
