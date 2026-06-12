# module2

Чистый рестарт: только ручное управление роботом, без автономки, без RTK, без звука.

## Структура

```
module2/
├── firmware/   # ESP32 (PlatformIO, Arduino framework)
└── app/        # Flutter (iOS/Android)
```

## Прошивка

```bash
cd firmware
pio run -t upload
pio device monitor
```

Что умеет:
- WiFi AP `Robot` (пароль `CHANGE_ME_MIN_8_CHARS`), IP `192.168.4.1`
- WebSocket на `/ws` (порт 81), HTTP `/ping` для health-check
- Hoverboard UART: `RX=16`, `TX=17`, 115200
- Реле: `GPIO32` (ATTACHMENT), `GPIO33` (MOUNT) — active HIGH
- Ramp 20 мс, failsafe 400 мс без команд → smooth stop
- Телеметрия батареи: `BAT_PCT,<pct>` каждые 500 мс

Протокол:
- `M,<left>,<right>` — управление, -70..70 на каждое колесо
- `STOP` — мгновенный smooth stop
- `ATTACHMENT_ON` / `ATTACHMENT_OFF`
- `MOUNT_ON` / `MOUNT_OFF`
- `PING` → `PONG`

## Приложение

```bash
cd app
flutter pub get
flutter run
```

Один экран: дифференциальный джойстик (Y = газ, X = рулёжка), кнопки STOP/ATTACH/MOUNT, индикатор батареи. Подключается к `ws://192.168.4.1:81/ws`.

## Зачем это

`sound/` работал, но имел много лишнего (I2S, LittleFS, файлы звуков). `rtk_firmware/` — автономка с RTK, BNO085, что для отладки управления избыточно. `module2` — минимальная база: руль, газ, реле, батарея. Всё, что нужно для проверки "управляется хорошо".
