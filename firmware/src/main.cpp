// =====================================================
//  Module2 v1: drive + 2 relays + battery telemetry
//  No sound, no RTK, no autonomous nav. Manual control only.
//  Pins (ESP32):
//   Hoverboard UART: RX=16  TX=17   (Serial2)
//   Relay ATTACHMENT = GPIO32  (active HIGH)
//   Relay MOUNT      = GPIO33  (active HIGH)
//
//  WiFi: STA mode (connects to your home router).
//  Create firmware/src/wifi_secrets.h with:
//     const char* WIFI_SSID = "your-ssid";
//     const char* WIFI_PASS = "your-password";
//  (file is in .gitignore — do not commit credentials)
// =====================================================

#include <stdint.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_system.h"

#if __has_include("wifi_secrets.h")
  #include "wifi_secrets.h"
#endif

// =======================
// Hoverboard serial protocol structs
// =======================
typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;

typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;

// =======================
// WiFi: STA mode. Credentials come from wifi_secrets.h
// (or fall back to placeholders below — build will succeed but ESP32
// will not connect).
// =====================================================
#ifndef WIFI_SSID
  #define WIFI_SSID  "CHANGE_ME"
#endif
#ifndef WIFI_PASS
  #define WIFI_PASS  "CHANGE_ME_TOO"
#endif

AsyncWebServer server(81);
AsyncWebSocket ws("/ws");

// =======================
// Pins
// =======================
static constexpr int PIN_HOVER_RX = 16;
static constexpr int PIN_HOVER_TX = 17;

static constexpr int PIN_RELAY_ATTACHMENT = 32; // active HIGH
static constexpr int PIN_RELAY_MOUNT      = 33; // active HIGH

// =======================
// Hoverboard serial protocol
// =======================
#define HOVER_SERIAL_BAUD 115200
#define START_FRAME       0xABCD

// =======================
// Drive tuning
// =======================
static constexpr int MAX_SPEED_PERCENT = 100;    // -100..100 raw scale (app now applies its own max%)
static constexpr int16_t HOVER_MAX_CMD = 300;    // hoverboard command scale
static constexpr int INPUT_DIV = 1;             // legacy — kept for compat; new app uses 4th arg

static constexpr uint32_t HOVER_SEND_MS  = 20;
static constexpr uint32_t CMD_TIMEOUT_MS = 400;

// ramp (percent domain) — gentler, smoother take-off
static constexpr uint32_t RAMP_UPDATE_MS = 40;
static constexpr int RAMP_STEP_UP_PER_TICK   = 1;   // ~25%/s acceleration
static constexpr int RAMP_STEP_DOWN_PER_TICK = 2;   // braking a bit snappier than accel

// extra smoothing in hoverboard command domain (small steps, not 4→2 jumps)
static constexpr int16_t SLEW_SPEED_PER_SEND = 2;
static constexpr int16_t SLEW_STEER_PER_SEND = 3;

int16_t g_cmdSpeed = 0;
int16_t g_cmdSteer = 0;

// =======================
// Battery telemetry
// =======================
static constexpr uint32_t BAT_SEND_MS = 500;
static constexpr float BAT_VOLT_SCALE = 0.01f;
static constexpr float TEMP_SCALE     = 0.1f;
static constexpr float BAT_FILTER_ALPHA = 0.25f;

static constexpr int CELL_COUNT = 10; // 10S
static constexpr int SEND_BAT_PCT_ONLY = 1; // UI
static constexpr int SEND_BAT_VERBOSE  = 1; // optional

// =======================
// WS client state
// =======================
static int g_wsClientCount = 0;
static uint32_t g_bootMs = 0;

// =======================
// Debug
// =======================
static constexpr uint32_t DEBUG_PRINT_MS = 200;
uint32_t g_lastDebugMs = 0;
int g_lastDbgL = 9999;
int g_lastDbgR = 9999;

// =======================
// WS buffer
// =======================
static constexpr size_t MAX_WS_MSG = 128;

// =======================
// Command state
// =======================
volatile int16_t g_targetLeft  = 0;
volatile int16_t g_targetRight = 0;
volatile int16_t g_maxPercent  = 100;   // set by app via M,L,R,MAX (10..100)
int16_t g_curLeft  = 0;
int16_t g_curRight = 0;

volatile uint32_t g_lastCmdMs = 0;
uint32_t g_lastSendMs = 0;
uint32_t g_lastRampMs = 0;
bool g_isFailSafeStopping = false;

// =======================
// Hoverboard feedback
// =======================
SerialFeedback Feedback{};
SerialFeedback NewFeedback{};

uint8_t  fb_idx = 0;
uint16_t fb_bufStartFrame = 0;
uint8_t* fb_p = nullptr;
uint8_t  fb_in = 0;
uint8_t  fb_in_prev = 0;

volatile bool g_haveFeedback = false;
uint32_t g_lastBatSendMs = 0;
float g_batVoltFiltered = 0.0f;

// =======================
// Helpers
// =======================
static inline int16_t clampi16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static inline int16_t stepToward16(int16_t cur, int16_t target, int16_t maxDelta) {
  int32_t diff = (int32_t)target - (int32_t)cur;
  if (diff >  maxDelta) diff =  maxDelta;
  if (diff < -maxDelta) diff = -maxDelta;
  return (int16_t)((int32_t)cur + diff);
}

static inline void trimInPlace(char* s) {
  int n = (int)strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == '\t')) s[--n] = 0;
  int i = 0;
  while (s[i] == ' ' || s[i] == '\r' || s[i] == '\n' || s[i] == '\t') i++;
  if (i > 0) memmove(s, s + i, strlen(s + i) + 1);
}

static inline bool streq(const char* a, const char* b) { return strcmp(a, b) == 0; }
static inline bool startsWith(const char* s, const char* pref) { return strncmp(s, pref, strlen(pref)) == 0; }

// =======================
// Relay control (active HIGH)
// =======================
void setAttachment(bool on) {
  digitalWrite(PIN_RELAY_ATTACHMENT, on ? HIGH : LOW);
  Serial.printf("ATTACHMENT %s\n", on ? "ON" : "OFF");
}

void setMount(bool on) {
  digitalWrite(PIN_RELAY_MOUNT, on ? HIGH : LOW);
  Serial.printf("MOUNT %s\n", on ? "ON" : "OFF");
}

// =======================
// SOC table (rough)
// =======================
static int socFromCellV(float vc) {
  const int N = 11;
  const float V[N] = {4.20f, 4.10f, 4.00f, 3.90f, 3.80f, 3.75f, 3.70f, 3.65f, 3.60f, 3.50f, 3.40f};
  const int   P[N] = { 100 ,   90 ,   80 ,   70 ,   60 ,   50 ,   40 ,   30 ,   20 ,   10 ,    0 };

  if (vc >= V[0]) return 100;
  if (vc <= V[N - 1]) return 0;

  for (int i = 0; i < N - 1; i++) {
    if (vc <= V[i] && vc >= V[i + 1]) {
      float t = (vc - V[i + 1]) / (V[i] - V[i + 1]);
      float pct = (float)P[i + 1] + t * (float)(P[i] - P[i + 1]);
      int out = (int)(pct + 0.5f);
      if (out < 0) out = 0;
      if (out > 100) out = 100;
      return out;
    }
  }
  return 0;
}

// =======================
// Hoverboard send
// =======================
void sendHoverboard(int16_t steer, int16_t speed) {
  SerialCommand cmd;
  cmd.start = (uint16_t)START_FRAME;
  cmd.steer = steer;
  cmd.speed = speed;
  cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);
  Serial2.write((uint8_t*)&cmd, sizeof(cmd));
}

void requestSmoothStop(const char* reason) {
  g_targetLeft = 0;
  g_targetRight = 0;
  Serial.printf("MOTORS: SMOOTH STOP (%s)\n", reason);
}

// left/right (-70..70) -> speed/steer -> SLEW -> send
void drive(int16_t leftPct, int16_t rightPct) {
  leftPct  = clampi16(leftPct,  -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  rightPct = clampi16(rightPct, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  int32_t speedT = (int32_t)(leftPct + rightPct) * (int32_t)HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);
  int32_t steerT = (int32_t)(rightPct - leftPct) * (int32_t)HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);

  int16_t spT = clampi16(speedT, -HOVER_MAX_CMD, HOVER_MAX_CMD);
  int16_t stT = clampi16(steerT, -HOVER_MAX_CMD, HOVER_MAX_CMD);

  g_cmdSpeed = stepToward16(g_cmdSpeed, spT, SLEW_SPEED_PER_SEND);
  g_cmdSteer = stepToward16(g_cmdSteer, stT, SLEW_STEER_PER_SEND);

  sendHoverboard(g_cmdSteer, g_cmdSpeed);
}

// =======================
// Ramp (percent domain)
// =======================
static inline int16_t stepTowardsLimited(int16_t cur, int16_t target, int maxDelta) {
  int diff = (int)target - (int)cur;
  if (diff >  maxDelta) diff =  maxDelta;
  if (diff < -maxDelta) diff = -maxDelta;
  return (int16_t)(cur + diff);
}

static inline int limitForAxis(int16_t cur, int16_t target, int ticks) {
  int absCur = abs((int)cur);
  int absTgt = abs((int)target);
  bool up = (absTgt > absCur);
  int step = up ? RAMP_STEP_UP_PER_TICK : RAMP_STEP_DOWN_PER_TICK;
  int maxDelta = ticks * step;
  if (maxDelta < 1) maxDelta = 1;
  return maxDelta;
}

void updateRamp() {
  uint32_t now = millis();
  uint32_t dt = now - g_lastRampMs;
  if (dt < RAMP_UPDATE_MS) return;

  uint32_t ticks = dt / RAMP_UPDATE_MS;
  g_lastRampMs += ticks * RAMP_UPDATE_MS;

  // Apply app's max speed cap (10..100) to the target before ramping.
  int16_t maxP = g_maxPercent;
  if (maxP < 10)   maxP = 10;
  if (maxP > 100)  maxP = 100;

  int32_t tLraw = (int32_t)g_targetLeft  * maxP / 100;
  int32_t tRraw = (int32_t)g_targetRight * maxP / 100;
  int16_t tL = clampi16(tLraw, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  int16_t tR = clampi16(tRraw, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  int limL = limitForAxis(g_curLeft,  tL, (int)ticks);
  int limR = limitForAxis(g_curRight, tR, (int)ticks);

  g_curLeft  = clampi16(stepTowardsLimited(g_curLeft,  tL, limL), -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  g_curRight = clampi16(stepTowardsLimited(g_curRight, tR, limR), -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  if (g_isFailSafeStopping && g_curLeft == 0 && g_curRight == 0 && tL == 0 && tR == 0) {
    g_isFailSafeStopping = false;
    Serial.println("FAILSAFE: STOPPED");
  }
}

// =======================
// Feedback receive
// =======================
void receiveHoverboardFeedback() {
  while (Serial2.available()) {
    fb_in = (uint8_t)Serial2.read();
    fb_bufStartFrame = ((uint16_t)fb_in << 8) | fb_in_prev;

    if (fb_bufStartFrame == START_FRAME) {
      fb_p = (uint8_t*)&NewFeedback;
      *fb_p++ = fb_in_prev;
      *fb_p++ = fb_in;
      fb_idx = 2;
    } else if (fb_idx >= 2 && fb_idx < sizeof(SerialFeedback)) {
      *fb_p++ = fb_in;
      fb_idx++;
    }

    if (fb_idx == sizeof(SerialFeedback)) {
      uint16_t cs = (uint16_t)(
        NewFeedback.start ^
        NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
        NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
        NewFeedback.batVoltage ^ NewFeedback.boardTemp ^
        NewFeedback.cmdLed
      );

      if (NewFeedback.start == START_FRAME && cs == NewFeedback.checksum) {
        memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
        g_haveFeedback = true;
      }
      fb_idx = 0;
    }
    fb_in_prev = fb_in;
  }
}

// =======================
// Battery to app
// =======================
void sendBatteryToAppIfNeeded() {
  if (!g_haveFeedback) return;

  uint32_t now = millis();
  if (now - g_lastBatSendMs < BAT_SEND_MS) return;
  g_lastBatSendMs = now;

  float vPack = (float)Feedback.batVoltage * BAT_VOLT_SCALE;
  float tempC = (float)Feedback.boardTemp * TEMP_SCALE;

  if (g_batVoltFiltered <= 0.01f) g_batVoltFiltered = vPack;
  g_batVoltFiltered = g_batVoltFiltered + BAT_FILTER_ALPHA * (vPack - g_batVoltFiltered);

  float vCell = (CELL_COUNT > 0) ? (g_batVoltFiltered / (float)CELL_COUNT) : 0.0f;
  int pct = socFromCellV(vCell);

  if (ws.count() > 0) {
    if (SEND_BAT_PCT_ONLY) {
      char sPct[24];
      snprintf(sPct, sizeof(sPct), "BAT_PCT,%d", pct);
      ws.textAll(sPct);
    }
    if (SEND_BAT_VERBOSE) {
      char out[160];
      snprintf(out, sizeof(out),
               "BAT,V=%.2fV,P=%d%%,Vf=%.2fV,temp=%.1fC",
               vPack, pct, g_batVoltFiltered, tempC);
      ws.textAll(out);
    }
  }
}

// =======================
// Debug wheel commands
// =======================
void debugPrintWheelSpeeds() {
  uint32_t now = millis();
  if (now - g_lastDebugMs < DEBUG_PRINT_MS) return;

  if (g_curLeft != g_lastDbgL || g_curRight != g_lastDbgR) {
    Serial.printf("LEFT CMD: %d   RIGHT CMD: %d\n", (int)g_curLeft, (int)g_curRight);
    g_lastDbgL = g_curLeft;
    g_lastDbgR = g_curRight;
  }
  g_lastDebugMs = now;
}

// =======================
// Parse "M,left,right[,max]"
// left/right: -100..100, max: 10..100 (optional, default 100)
// =======================
bool parseMove(const char* msg, int16_t& outL, int16_t& outR, int16_t& outMax) {
  if (!startsWith(msg, "M,")) return false;

  const char* p = msg + 2;
  char* end1 = nullptr;
  long L = strtol(p, &end1, 10);
  if (!end1 || *end1 != ',') return false;

  const char* p2 = end1 + 1;
  char* end2 = nullptr;
  long R = strtol(p2, &end2, 10);
  if (!end2) return false;

  outMax = 100; // default
  if (*end2 == ',') {
    const char* p3 = end2 + 1;
    char* end3 = nullptr;
    long M = strtol(p3, &end3, 10);
    if (end3 && M >= 10 && M <= 100) {
      outMax = (int16_t)M;
    }
  }

  outL = clampi16(L, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  outR = clampi16(R, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  return true;
}

// =======================
// WebSocket handler
// =======================
void onWsEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS CONNECT id=%u ip=%s\n", client->id(), client->remoteIP().toString().c_str());
    client->text("STATE,CONNECTED");

    g_isFailSafeStopping = false;
    g_wsClientCount++;

    if (g_wsClientCount == 1 && (millis() - g_bootMs) > 1500) {
      // first connect after boot — nothing to do without sound, but keep slot
    }
    return;
  }

  if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS DISCONNECT id=%u -> SMOOTH STOP\n", client->id());
    requestSmoothStop("ws_disconnect");
    if (g_wsClientCount > 0) g_wsClientCount--;
    return;
  }

  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;

    if (!info->final || info->index != 0 || info->len != len) return;
    if (info->opcode != WS_TEXT) return;
    if (len >= MAX_WS_MSG) { client->text("ERR,TOO_LONG"); return; }

    char msg[MAX_WS_MSG];
    memcpy(msg, data, len);
    msg[len] = 0;
    trimInPlace(msg);

    g_lastCmdMs = millis();
    g_isFailSafeStopping = false;

    // ---- ping/stop
    if (streq(msg, "PING")) { client->text("PONG"); return; }
    if (streq(msg, "STOP")) { requestSmoothStop("STOP_cmd"); client->text("OK STOP"); return; }

    // ---- movement
    int16_t L=0, R=0, M=100;
    if (parseMove(msg, L, R, M)) {
      g_targetLeft = L;
      g_targetRight = R;
      g_maxPercent = M;
      client->text("OK M");
      return;
    }

    // ---- relays from app
    if (streq(msg, "ATTACHMENT_ON"))  { setAttachment(true);  client->text("OK ATTACHMENT_ON");  return; }
    if (streq(msg, "ATTACHMENT_OFF")) { setAttachment(false); client->text("OK ATTACHMENT_OFF"); return; }
    if (streq(msg, "MOUNT_ON"))       { setMount(true);       client->text("OK MOUNT_ON");       return; }
    if (streq(msg, "MOUNT_OFF"))      { setMount(false);      client->text("OK MOUNT_OFF");      return; }

    client->text("ERR,UNKNOWN");
  }
}

// =======================
// Setup / Loop
// =======================
void setup() {
  Serial.begin(115200);
  delay(200);

  g_bootMs = millis();
  Serial.printf("\n=== Module2 v1 START | reset reason=%d ===\n", (int)esp_reset_reason());

  // relays
  pinMode(PIN_RELAY_ATTACHMENT, OUTPUT);
  pinMode(PIN_RELAY_MOUNT, OUTPUT);
  setAttachment(false);
  setMount(false);

  // hoverboard
  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, PIN_HOVER_RX, PIN_HOVER_TX);
  Serial.printf("HoverSerial: %d baud, RX=%d TX=%d\n", HOVER_SERIAL_BAUD, PIN_HOVER_RX, PIN_HOVER_TX);

  // WiFi STA — connect to home router
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi: connecting to %s", WIFI_SSID);
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStart > 20000) {
      Serial.println(" FAILED (timeout 20s)");
      Serial.println("Rebooting in 3s — check wifi_secrets.h");
      delay(3000);
      ESP.restart();
    }
    Serial.print('.');
    delay(500);
  }
  Serial.println();
  Serial.print("WiFi: connected, IP=");
  Serial.println(WiFi.localIP());
  Serial.print("Signal: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("HTTP+WS server on port 81, WS path /ws");
  Serial.println("Commands: M,left,right | STOP | ATTACHMENT_ON/OFF | MOUNT_ON/OFF");

  g_lastCmdMs = millis();
  g_lastSendMs = millis();
  g_lastRampMs = millis();
  g_lastDebugMs = millis();
  g_lastBatSendMs = millis();

  g_targetLeft = 0;
  g_targetRight = 0;
  g_curLeft = 0;
  g_curRight = 0;

  g_cmdSpeed = 0;
  g_cmdSteer = 0;

  drive(0, 0);
  Serial.println("READY");
}

void loop() {
  ws.cleanupClients();

  // WiFi watchdog — keep STA alive
  static uint32_t lastWifiCheckMs = 0;
  if (millis() - lastWifiCheckMs > 10000) {
    lastWifiCheckMs = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi: lost connection, reconnecting…");
      WiFi.reconnect();
    }
  }

  const uint32_t now = millis();

  // hoverboard feedback
  receiveHoverboardFeedback();

  // failsafe -> smooth stop
  if (now - g_lastCmdMs > CMD_TIMEOUT_MS) {
    if (!g_isFailSafeStopping && (g_targetLeft != 0 || g_targetRight != 0 || g_curLeft != 0 || g_curRight != 0)) {
      g_isFailSafeStopping = true;
      requestSmoothStop("timeout");
    }
  }

  // ramp
  updateRamp();

  // send drive cmd
  if (now - g_lastSendMs >= HOVER_SEND_MS) {
    g_lastSendMs = now;
    drive(g_curLeft, g_curRight);
  }

  // battery telemetry
  sendBatteryToAppIfNeeded();

  // debug
  debugPrintWheelSpeeds();
}
