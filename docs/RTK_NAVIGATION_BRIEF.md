# RTK-навигация: глубокий разбор для модуля 2

> Документ — выжимка из четырёх исследований. Цель: понять, что нужно, чтобы робот с ESP32 + ZED-F9P + BNO085 ездил по waypoint'ам автономно. Документ не запускает реализацию; он даёт фундамент для принятия решения.

---

## 0. Общая картина (как робот вообще понимает, где он)

Чтобы робот поехал из точки A в точку B автономно, нужна замкнутая цепочка из 4 стадий. Каждая с провалами и компромиссами.

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│ SENSE    │ →  │ LOCALIZE │ →  │ PLAN     │ →  │ ACT      │
│ (GPS+IMU)│    │ (EKF)    │    │ (waypts) │    │ (control)│
└──────────┘    └──────────┘    └──────────┘    └──────────┘
     ↑                ↑               ↑              │
     │                │               │              │
     └──── телеметрия в приложение ←───┴──────────────┘
```

**SENSE**: сырые измерения — GPS (lat/lon/accuracy), IMU (accel/gyro/mag), wheel encoders (если есть).
**LOCALIZE**: где я **точно** сейчас, с какой погрешностью. Это фьюжн-фильтр (EKF).
**PLAN**: в какую сторону ехать. Алгоритм преследования (pure pursuit).
**ACT**: команды на моторы. PID.

### Что значит "верно ли он едет"?

Вопрос разбивается на 3 проверки:

1. **Позиция соответствует GPS-фиксу** — если RTK_FIX, ошибка 1–2 см; если RTK_FLOAT, 10–30 см; если вообще GPS пропал, drift IMU.
2. **Heading (куда смотрит) совпадает с целевым** — измеряется GPS-скоростью (только при v > 1.5 м/с) или магнитометром.
3. **Пройденный путь соответствует командам** — wheel encoders + IMU, опционально.

Ответ "верно ли" даёт **EKF**: он оценивает полное состояние с ковариацией, и если ковариация позиции > 5 м — это сигнал "не верь, стоп".

---

## 1. BNO085 + GPS sensor fusion (полный отчёт)

### 1.1 BNO085 — sensor hub

BNO085 — IMU со встроенным sensor fusion. Сам делает quaternion'ы и выдаёт готовые:

- `Game Rotation Vector` (без магнитометра) — для outdoor
- `Rotation Vector` (с магнитометром) — для indoor
- `Linear Acceleration` (без гравитации)
- `Gravity` (только гравитация)

Для outdoor-робота — **Game Rotation Vector**: магнитометр вблизи hoverboard-моторов и ESC сильно искажён.

**Протокол SHTP** (Sensor Hub Transport Protocol):
```
| Length LSB | Length MSB | Channel | Sequence | Data... |
| 1 byte     | 1 byte     | 1 byte  | 1 byte   | N bytes |
```

Каждый сенсор нужно "включить" командой `Set Feature Command` (channel 2, opcode 0xFD), с указанием report_interval_us.

### 1.2 Quaternion → Euler

```c
void quat_to_euler(const float q[4], float euler[3]) {
    // q = [w, x, y, z]
    float sinr_cosp = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
    euler[0] = atan2f(sinr_cosp, cosr_cosp);  // roll

    float sinp = 2.0f * (q[0]*q[2] - q[3]*q[1]);
    if (fabsf(sinp) >= 1.0f) euler[1] = copysignf(M_PI/2, sinp);
    else                    euler[1] = asinf(sinp);

    float siny_cosp = 2.0f * (q[0]*q[3] + q[1]*q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
    euler[2] = atan2f(siny_cosp, cosy_cosp);  // yaw
}
```

### 1.3 Heading drift

Game Rotation Vector **не держит абсолютный heading дольше нескольких минут** (по community reports — порядка 1°/min у неподвижного, больше в движении). Нужно корректировать из GPS-скорости.

### 1.4 Сравнение фильтров

| Метод | Сложность | Точность | Нужна модель шума | Где |
|---|---|---|---|---|
| Complementary (α-filter) | очень низкая | средняя | нет | roll/pitch |
| Madgwick | низкая | хорошая | частично | IMU-only |
| Mahony | низкая | хорошая | частично | IMU-only |
| EKF (full state) | высокая | высокая | да (Q, R) | **GPS+IMU** |

**Вывод**: для нашего робота — **EKF 9-15 state** (см. ниже).

### 1.5 Madgwick filter (готовый код)

```c
typedef struct {
    float q[4];      // [w, x, y, z]
    float beta;      // gradient descent gain (~0.033)
    float dt;
} madgwick_t;

void madgwick_init(madgwick_t *f, float dt, float beta) {
    f->q[0] = 1.0f; f->q[1] = f->q[2] = f->q[3] = 0.0f;
    f->beta = beta; f->dt = dt;
}

void madgwick_update(madgwick_t *f,
                     float gx, float gy, float gz,    // gyro, rad/s
                     float ax, float ay, float az,    // accel
                     float mx, float my, float mz) {  // mag
    // ... полная формула в research-результатах
    // Step: gradient descent + gyro integration + renormalize
}
```

### 1.6 EKF для GPS+IMU (9-state минимум, 15-state полный)

**State vector (15-state из ArduPilot):**
$$
\mathbf{x} = [\mathbf{p} \; \mathbf{v} \; \mathbf{q} \; \mathbf{b}_a \; \mathbf{b}_g] \in \mathbb{R}^{15}
$$

**Prediction (IMU, 100 Hz)** для differential drive:
$$
\begin{aligned}
x_{k+1} &= x_k + v_k \cos\psi_k \cdot \Delta t \\
y_{k+1} &= y_k + v_k \sin\psi_k \cdot \Delta t \\
\psi_{k+1} &= \psi_k + \omega_k \cdot \Delta t
\end{aligned}
$$

**Jacobian F:**
$$
F = \begin{bmatrix}
1 & 0 & \cos\psi\cdot\Delta t & -\sin\psi\cdot\Delta t & 0 \\
0 & 1 & \sin\psi\cdot\Delta t & \cos\psi\cdot\Delta t  & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

**Update (GPS, 5–10 Hz):**
$$
K = P H^T (H P H^T + R)^{-1}
$$
$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + K (\mathbf{z}_{GPS} - h(\hat{\mathbf{x}}))
$$

**Численная стабильность на ESP32**:
- **Joseph form** для обновления P: $P = (I-KH)P(I-KH)^T + KRK^T$ — дороже, но стабильнее
- Каждые N шагов **symmetrize**: $P = (P + P^T)/2$
- Covariance reset если $\text{trace}(P) < 0$ или NaN
- **double** для матриц (ESP32 soft-FPU ~6 циклов на double, всё равно ОК)

### 1.7 Dead reckoning

**Без wheel encoders** (как у нас сейчас) — IMU accel integration даёт drift = $\tfrac{1}{2}b \cdot t^2$ и расходится за секунды. Нужно:
- **GPS speed + GPS heading** при v > 1.5 m/s — корректируют IMU
- Между GPS update'ами — экстраполяция heading по gyro, скорости по последнему GPS speed
- **Вывод**: ставить **wheel encoders** ($1 за оптопару) — радикально улучшит ситуацию

### 1.8 Heading estimation — какой источник когда

| Скорость GPS | Источник heading | $R_\psi$ (measurement noise) |
|---|---|---|
| v < 0.5 m/s | только магнитометр | 10° |
| 0.5 < v < 1.5 m/s | интерполяция | среднее |
| v > 1.5 m/s | GPS speed heading | 1–2° |

GPS speed heading: $\psi = \text{atan2}(v_E, v_N)$. ZED-F9P при RTK даёт velocity с точностью 0.05 m/s, что на 1.5 m/s → ~1° heading.

### 1.9 Initial alignment

- **Roll/pitch**: ~1 сек неподвижно, чтобы accel усреднился
- **Yaw (без магнитометра)**: произвольный; задать вручную или взять GPS heading при первом движении
- **Gyro bias**: BNO085 калибрует сам, 3-5 сек неподвижно улучшают

---

## 2. ZED-F9P + NTRIP (RTK)

### 2.1 Принцип RTK

Carrier-phase differential GPS. Base измеряет фазу несущей, считает ошибки, отдаёт rover'у по радио/WiFi/Internet. Rover корректирует свои измерения, **фиксит integer ambiguity** → сантиметровая точность.

**Фиксы** (от худшего к лучшему):
- NONE — нет сигнала
- SINGLE — 5-10 м, как обычный GPS
- PSRDIFF — 1-3 м, WAAS/EGNOS
- RTK_FLOAT — 10-30 см, ambiguities не разрешены
- **RTK_FIX** — **1-2 см** по горизонтали
- PPP — ещё точнее, но нужно интернет (PointPerfect)

### 2.2 Base station — Survey-In vs Fixed

| Режим | Когда | Точность base coords |
|---|---|---|
| Survey-In | "поставил и забыл" | 2-5 м (если 60-300 сек, acc 2-5 м) |
| Fixed | "знаю точно где" | 0 (забито вручную) |

Для нашего проекта — **Survey-In** на 120 сек с acc 3 м — даст 2-3 м точку base, чего хватит для относительного RTK (1-2 см между base и rover).

### 2.3 RTCM3 messages (что гнать)

Обязательные:
- **1005** — base ARP, antenna height
- **1074** — MSM4 GPS
- **1084** — MSM4 GLONASS
- **1094** — MSM4 Galileo
- **1124** — MSM4 BeiDou
- **1230** — GLONASS code-phase biases

Современный способ настройки — `CFG-VALSET` (32-bit key IDs, начиная с protocol v23), не старые `CFG-MSG`.

### 2.4 NTRIP

- **NTRIP 1.0/2.0** — HTTP-подобный протокол для раздачи RTCM3
- Rover шлёт **GGA** (своя позиция) → caster
- Caster шлёт **RTCM3** (поправки) → rover
- Caster: бесплатный (RTK2GO, EMLID, SNIP) или свой (на базе `str2str` из RTKLIB)
- TLS опционально, HTTP basic auth

**NTRIP client на ESP32** — обычный HTTP, в худшем случае ~30 строк кода. Но TLS + разбор GGA/RTCM3 — не тривиально на ESP32 с ограниченной RAM.

### 2.5 UBX-конфигурация (вкратце)

```python
# pyubx2 пример
from pyubx2 import UBXMessage, SET, VALSET
msg = UBXMessage('CFG', 'CFG-VALSET', SET,
                 layers=1,  # RAM only (volatile)
                 cfgData={...})  # 32-bit keys
```

Ключи:
- `CFG_SFMODE` — Survey-In / Fixed
- `CFG_SVIN_MIN_DUR` — минимум длительности (сек)
- `CFG_SVIN_ACC_LIMIT` — целевая точность (м)
- `CFG_MSGOUT_RTCM_3X_TYPE*` — какие RTCM3 messages слать
- `CFG_RATE_MEAS` — 100 мс = 10 Hz (максимум для ZED-F9P)

### 2.6 Что делать при потере RTK_FIX

- **5+ секунд** без fix → деградировать к RTK_FLOAT, не паниковать
- **30+ секунд** → визуальный/звуковой alert в GCS
- **5+ минут без GPS вообще** → dead reckoning по IMU, переход в "blind" режим
- **Возврат RTK_FIX** → re-converge, фикс восстановится за 1-5 сек

### 2.7 Источники

- u-blox ZED-F9P Interface Description (актуальная версия v27.30+)
- u-blox Application Note: "RTK Reference Design with ZED-F9P"
- RTKLIB `str2str` для NTRIP server mode
- Ardusimple "How to configure ZED-F9P as base" tutorial
- SparkFun RTK Surveyor hookup guide
- `pyubx2` / `pynmeagps` Python library

---

## 3. Control algorithms (pure pursuit vs Stanley)

### 3.1 Pure Pursuit

**Тип**: чисто геометрический, без модели робота. **Outputs**: linear + angular velocity напрямую.
**Параметр**: lookahead distance $L_d$ (фиксированный или адаптивный).
**Алгоритм**:
1. Найти ближайшую точку на пути до робота
2. Найти точку на пути на расстоянии $L_d$ от робота (lookahead)
3. Перевести lookahead-точку в body frame
4. $\omega = 2 v \sin\alpha / L_d$, где $\alpha$ — heading error
5. $v$ — фиксированная или speed-limited по curvature

**Плюсы**: прост, robust, нативно для differential drive.
**Минусы**: cutting corners, нет explicit heading term, может осциллировать на высокой скорости.

**Для нашего робота — рекомендую Pure Pursuit**. Stanley требует bicycle model (рулевой угол), адаптация к diff-drive = переписать в $(v, \omega)$, что сводится к варианту pure pursuit.

### 3.2 Stanley (упоминаю для полноты)

Использует cross-track error и heading error:
$$
\delta(t) = \psi(t) + \arctan\left(\frac{k \cdot e_{\text{fa}}}{v(t)}\right)
$$
где $e_{\text{fa}}$ — ошибка в front-axle. Для diff-drive — адаптация неуклюжая, **не рекомендую**.

### 3.3 Альтернативы

- **Follow-the-carrot** — простейший, target = ближайшая waypoint, ехать туда
- **Line-of-sight (LOS)** — морская навигация, для surface-роботов
- **VFH / DWA** — локальный obstacle avoidance (не у нас, нет лидара)
- **Teb local planner** — trajectory optimization, требует ROS2

### 3.4 PID для моторов (отдельно от path-follower)

Внутри pure pursuit'а есть **inner loop** — управление скоростью колёс. Это обычный PID:
- **Single PID**: error = target_speed - measured_speed (из wheel encoders)
- **Anti-windup**: clamp integral
- **Derivative filter**: low-pass на входе
- **Tuning**: Ziegler-Nichols или software-in-the-loop

У нас моторы уже рулятся через hoverboard UART (steer, speed в команд-домене). Это **заменяет PID** — встроенный в плату контроллер делает токовое управление.

### 3.5 Источники

- R. C. Coulter 1992, "Implementation of the Pure Pursuit Path Tracking Algorithm"
- Stanford DARPA Junior — Stanley controller paper
- ArduPilot Rover source — `Rover/control_mode.cpp`, `pure_pursuit.cpp`
- github: thien94/YOLO_ros_rover (дифф. привод + pure pursuit)
- Pure pursuit variants: github: AtsushiSakai/PythonRobotics

---

## 4. Flutter Ground Control Station

### 4.1 Карты

`flutter_map` ^8.3.0 — **основной выбор**. OpenStreetMap, без ключей, без лимитов.

```yaml
dependencies:
  flutter_map: ^8.3.0
  latlong2: ^0.9.1
  geolocator: ^13.0.0  # для текущей позиции телефона
```

Минимальный пример:
```dart
FlutterMap(
  options: const MapOptions(
    initialCenter: LatLng(55.7558, 37.6173),
    initialZoom: 18,  // для outdoor waypoint nav — zoom 18-19
  ),
  children: [
    TileLayer(
      urlTemplate: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
      userAgentPackageName: 'com.module2',
    ),
    MarkerLayer(markers: [...]),
    PolylineLayer(polylines: [...]),  // траектория
  ],
)
```

### 4.2 Offline тайлы

`flutter_map_tile_caching` ^1.4.0 — кэширование FMTC store:
```dart
import 'package:flutter_map_tile_caching/flutter_map_tile_caching.dart';
TileLayer(
  urlTemplate: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
  userAgentPackageName: 'com.module2',
  tileProvider: FMTCStore('mapStore').getTileProvider(),
)
```

Продвинутый путь: **MBTiles / PMTiles** — единый файл с тайлами, скачивается заранее по bbox.

### 4.3 iOS Info.plist

Обязательно:
```xml
<key>NSLocationWhenInUseUsageDescription</key>
<string>Показываем твоё местоположение и карту вокруг робота</string>
<key>NSLocationAlwaysUsageDescription</key>
<string>Фоновое отслеживание для записи трека</string>
```

`NSAllowsLocalNetworking` (мы его уже добавили) — для WebSocket к ESP32.

### 4.4 Телеметрия (формат)

JSON-сообщения (от ESP32 → приложение) — расширение текущего протокола:

```
POS,<lat>,<lon>,<alt_m>,<accuracy_m>,<fix_type>,<heading_deg>,<speed_mps>
SATS,<used>,<visible>,<hdop>
EST,<state>,<x_est>,<y_est>,<heading_est>,<pos_cov_xx>,<pos_cov_yy>
WP,<idx>,<total>,<dist_to_next>,<eta_s>
FOLLOW,<target_x>,<target_y>,<cross_track>,<v_cmd>,<omega_cmd>
BAT,<pct>,<voltage>,<temp_c>
```

`fix_type`: 0=NONE, 1=DR, 2=2D, 3=3D, 4=GPS+DR, 5=RTK_FLOAT, 6=RTK_FIX
`state`: IDLE, ARMED, FOLLOWING, PAUSED, GOAL_REACHED, FAULT

### 4.5 Mission planning UI

- **Рисование waypoint'ов**: tap на карте → добавить точку в список
- **Drag/drop**: long press → перетаскивание
- **Edit**: диалог с координатами, alt, dwell time
- **Geofence**: polygon drawing (отдельный `PolygonLayer`)
- **Survey pattern**: lawn-mower (вперёд-разворот-вперёд), perimeter
- **Import/export**: KML, GeoJSON, MAVLink `.waypoints` файл

### 4.6 Failsafe UI

- Banner сверху: "RTK FIX lost, FLOAT for 12s" — цвет меняется
- Mission pause button (большая кнопка)
- "Return to launch" RTL
- Connection lost overlay — большая красная полоса
- Manual override: переключатель "AUTO/MANUAL" — мгновенный переход к джойстику

### 4.7 Источники

- https://pub.dev/packages/flutter_map — версия 8.3.0
- https://docs.fleaflet.com — документация
- https://github.com/fleaflet/flutter_map — исходники
- `geolocator` ^13.0.0 — для location permissions
- `flutter_map_tile_caching` ^1.4.0 — offline тайлы
- QGroundControl (Qt GCS) — open source, reference UI
- Ardupilot mission file format

---

## 5. Архитектура высокого уровня (high-level plan)

### 5.1 Компоненты

```
┌─────────────────────────────────────────────────────────┐
│ ESP32 (модуль 2)                                        │
│ ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐ │
│ │ Hoverboard   │  │ ZED-F9P UART │  │ BNO085 I2C       │ │
│ │ UART 115200  │  │ (RTK rover)  │  │ (Game Rot Vec)   │ │
│ └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘ │
│        │                 │                   │           │
│        ▼                 ▼                   ▼           │
│ ┌─────────────────────────────────────────────────────┐ │
│ │ NavigationCore (новый модуль)                       │ │
│ │  - SensorHub: parse BNO085                          │ │
│ │  - GPSParser: parse UBX/NMEA                        │ │
│ │  - NTRIPClient: GGA in / RTCM3 out                  │ │
│ │  - EKF: state estimate (x, y, v, ψ, b)              │ │
│ │  - WaypointQueue: список waypoint'ов                │ │
│ │  - PurePursuit: control output                      │ │
│ │  - StateMachine: IDLE→ARMED→FOLLOWING→GOAL→FAULT   │ │
│ └─────────────────────┬───────────────────────────────┘ │
│                       │                                 │
│ ┌─────────────────────▼───────────────────────────────┐ │
│ │ Telemetry (WebSocket) — расширить существующий      │ │
│ │  + M,left,right,max (уже есть)                      │ │
│ │  + AUTO,<state>,<x>,<y>,<heading>,<speed>           │ │
│ │  + POS,<lat>,<lon>,<acc>,<fix>                      │ │
│ │  + WP,<idx>,<total>,<dist>                          │ │
│ │  + FOLLOW,<v_cmd>,<ω_cmd>                           │ │
│ └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### 5.2 Последовательность старта

1. ESP32 boot → WiFi connect → WebSocket ready
2. BNO085 init → 3-5 сек gyro calibration
3. ZED-F9P init → 30-60 сек cold start → SINGLE fix
4. NTRIP client connect → GGA → RTCM3 stream → RTK_FIX (1-5 мин)
5. App подключается, видит `STATE,READY,fix=RTK_FIX,sats=18,hdop=0.6`
6. App отправляет mission: `MISSION,<n> <lat1>,<lon1>,<alt1>,<speed1>; <lat2>,<lon2>,...`
7. ESP32 загружает waypoint'ы, ждёт команду `ARM`
8. App → `ARM` → StateMachine: IDLE → ARMED
9. App → `START` → PurePursuit активируется
10. Робот едет, шлёт POS/FOLLOW каждые 100 мс
11. App показывает траекторию на карте
12. При достижении последнего waypoint → `GOAL_REACHED`
13. App → `DISARM` → StateMachine: ARMED → IDLE

### 5.3 Что нужно докупить (BOM, не исчерпывающий)

| Компонент | Где | Цена | Зачем |
|---|---|---|---|
| ZED-F9P модуль | Ardusimple, SparkFun | $80-200 | RTK (у нас есть?) |
| BNO085 breakout | SparkFun, Adafruit | $30-50 | IMU (у нас есть?) |
| Wheel encoders (оптопары + диски) | AliExpress | $5 | dead reckoning |
| Surveyor-grade антенна | Ardusimple | $40-100 | для base |
| NTRIP caster доступ | EMLID, RTK2GO | бесплатно | RTCM3 поток |

Уточни, что уже есть из этого.

### 5.4 Трудозатраты (грубая оценка)

| Этап | Что | Время |
|---|---|---|
| **Прототип на столе** | Подключить ZED-F9P + BNO085 к ESP32, парсить UBX/SHTP, лить в Serial | 1-2 дня |
| **NTRIP client** | Подключиться к бесплатному caster'у, видеть RTK_FIX | 1 день |
| **EKF** | 9-state EKF, верификация на записанных логах | 3-5 дней |
| **Pure pursuit + state machine** | Waypoint'ы, ARM/START/DISARM | 2-3 дня |
| **Flutter GCS** | Карта, телеметрия, mission planning | 5-7 дней |
| **Field testing** | Outdoor, RTK fix, e2e проезд | 2-3 дня |
| **Итого** | | **2-3 недели** |

Если первый раз — 4 недели. Если уже есть опыт — 1.5-2.

### 5.5 Что не нашёл / нужно дополнительно изучить

- **Точная цифра дрифта BNO085 Game Rotation Vector** — оценочно ~1°/min, но не из datasheet. В datasheet — гироскоп noise density и bias stability, но не интегральный yaw drift. Если важна абсолютная точность heading — поставить dual-antenna GPS (overkill).
- **Реальные Q, R параметры для ESP32 EKF** — брать из ArduPilot EKF docs, адаптировать под наш hoverboard.
- **NTRIP TLS** — на ESP32 не тривиально (mbedTLS overhead). Если caster без TLS — лучше.
- **Pure pursuit lookahead tuning** — эмпирически, начать с L_d = 1.5-2 м для нашего размера робота.
- **flutter_map 8.3.0** — актуально на 2025-2026, но Flutter SDK требует ≥ 3.0, проверить совместимость.

---

## 6. Что я предлагаю как следующий шаг

Прежде чем внедрять всё это вслепую:

1. **Проверить, что у тебя физически есть**: ZED-F9P, BNO085, антенны, wheel encoders. Без этого половина документа — теория.

2. **Минимальный e2e тест**:
   - Подключить ZED-F9P к ESP32 по UART
   - Парсить UBX NAV-PVT, лить в Serial
   - В Serial Monitor увидеть lat/lon/fix_type/sats
   - **Без** RTK, без NTRIP, без EKF — просто "GPS работает"

3. **Если работает** — постепенно добавлять:
   - NTRIP client
   - BNO085 + EKF
   - Waypoint nav
   - Flutter GCS

4. **Если не работает** — дебажить base, антенны, видимость неба.

Не пытайся делать всё сразу. Сначала "GPS говорит, где я" — это неделя. Потом "GPS говорит точно, где я" — ещё неделя. Потом "робот едет туда" — ещё две.

---

## Источники (все, одним списком)

**RTK / ZED-F9P / NTRIP**:
- u-blox ZED-F9P Interface Description (v27.30+)
- u-blox Application Note: "RTK Reference Design with ZED-F9P"
- RTKLIB `str2str` documentation
- Ardusimple ZED-F9P base tutorial
- SparkFun RTK Surveyor hookup guide
- pyubx2 / pynmeagps Python libraries

**Sensor fusion / BNO085 / EKF**:
- CEVA/Hillcrest Labs BNO080/085 datasheet + SH-2 reference manual
- Madgwick 2010 paper (xioTechnologies)
- Mahony 2008 paper (UQ ITEE)
- Sola 2017 "Quaternion kinematics for ESKF" (arxiv 1711.02508)
- ArduPilot EKF docs
- ROS robot_localization package
- SparkFun BNO080 Arduino Library

**Control algorithms**:
- R. C. Coulter 1992, "Implementation of Pure Pursuit Path Tracking Algorithm"
- Stanford DARPA Junior — Stanley controller
- ArduPilot Rover source
- github: thien94/YOLO_ros_rover
- github: AtsushiSakai/PythonRobotics
- "A Survey of Motion Planning and Control Techniques for Self-Driving Vehicles" (2024)

**Flutter GCS**:
- https://pub.dev/packages/flutter_map (v8.3.0)
- https://docs.fleaflet.com
- https://github.com/fleaflet/flutter_map
- geolocator ^13.0.0
- flutter_map_tile_caching ^1.4.0
- QGroundControl source (Qt GCS, reference)
- Ardupilot mission file format
