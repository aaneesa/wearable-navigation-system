// Wearable navigation | Node B: touch and LEDs | ESP32-S3, Arduino-ESP32 3.x
// Serial Monitor: 115200 baud. Send h for controls.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Both boards use the same small, versioned, mode-independent packet.
enum Sensor : uint8_t { LEFT, CENTER, RIGHT, UP, DOWN, BEAMS };
enum Direction : uint8_t { HOLD, FORWARD, TURN_LEFT, TURN_RIGHT };
enum Reason : uint8_t { CALIBRATE, CHECK_GROUND, NO_OPEN_BEAM, PATH_FOUND };

constexpr uint8_t CHANNEL = 1;
constexpr uint16_t MAGIC = 0x4E56;
constexpr uint8_t VERSION = 2;
constexpr uint16_t FRESH_MS = 450;
constexpr uint16_t LINK_MS = 300;
constexpr uint16_t AIR_MARGIN_MS = 30;  // Bench allowance; measure on the belt.

struct __attribute__((packed)) Packet {
  uint32_t boot, seq;
  uint16_t magic;
  uint8_t version, direction, reason, warning, clear;
  uint16_t lifeMs[BEAMS];  // Remaining evidence lifetime at transmission.
};
static_assert(sizeof(Packet) == 25, "Check packet layout on both boards");

constexpr uint8_t bitFor(unsigned i);
constexpr uint8_t bitFor(unsigned i) { return uint8_t(1U << i); }
constexpr uint8_t FRONT = bitFor(CENTER) | bitFor(UP);
// The application tasks subscribe to the existing watchdog when available.
void startWatchdog() {
  if (esp_task_wdt_status(nullptr) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_config_t config = {};
    config.timeout_ms = 5000;
    config.trigger_panic = true;
    esp_task_wdt_init(&config);
  }
}

bool watchThisTask() {
  bool active = esp_task_wdt_status(nullptr) == ESP_OK || esp_task_wdt_add(nullptr) == ESP_OK;
  if (!active) Serial.println("Watchdog unavailable for this task; check board settings.");
  return active;
}

void fatal(const char *message) {
  Serial.println(message);
  delay(1500);
  ESP.restart();
}

constexpr uint8_t LED_PIN[] = {5, 6, 7, 8}; // Forward, left, right, down/hold
constexpr uint8_t BUTTON[] = {9, 10};       // Active-HIGH external touch modules
constexpr uint8_t LED_ACTIVE = HIGH, TOUCH_ACTIVE = HIGH;
constexpr uint8_t EXPECT_A[6] = {};  // Enter A's STA MAC to bind; zeros learn first valid sender per boot.
constexpr uint8_t ALL_LEDS = 15;
constexpr uint16_t DEBOUNCE_MS = 50;
enum View : uint8_t { LINK_LOST, SENSORS_UNAVAILABLE, OBSTACLES, DIRECTION, PAUSE, LAMP_TEST };
struct Received { Packet p; uint32_t at; };
struct Display {
  View view = LINK_LOST;
  uint8_t leds = 0, direction = HOLD, reason = CALIBRATE, valid = 0, warnings = 0;
};
struct Status {
  Display display{};
  uint8_t mode = 1, source[6]{};
  uint32_t seq = 0, age = 0, maxTickUs = 0, touches = 0, rejected = 0;
  bool paired = false;
};
QueueHandle_t inbox, uiCommands, monitorBox;

// Only the Wi-Fi receive callback writes receiver state. The UI gets copies.
struct Receiver {
  uint8_t mac[6]{};
  bool paired = false, synced = false, candidate = false;
  uint32_t boot = 0, seq = 0, at = 0, nextBoot = 0, nextSeq = 0, nextAt = 0;
  uint32_t rejected = 0;
  bool accept(const uint8_t *src, const Packet &p, uint32_t now) {
    const uint8_t zero[6]{};
    if (memcmp(EXPECT_A, zero, 6) && memcmp(src, EXPECT_A, 6)) return false;
    if (paired && memcmp(src, mac, 6)) return false;
    if (!paired) { memcpy(mac, src, 6); paired = true; }
    if (synced && p.boot == boot && int32_t(p.seq - seq) <= 0) return false;
    if (!synced || p.boot != boot || now - at >= LINK_MS) {
      if (synced && now - at < LINK_MS) return false;
      // Require two increasing packets after startup or a sender reboot.
      if (!candidate || p.boot != nextBoot || now - nextAt >= LINK_MS) {
        candidate = true; nextBoot = p.boot; nextSeq = p.seq; nextAt = now;
        return false;
      }
      if (int32_t(p.seq - nextSeq) <= 0) return false;
    }
    synced = true; candidate = false;
    boot = p.boot; seq = p.seq; at = now;
    return true;
  }
};
Receiver receiver;
struct Message { Received rx; uint8_t source[6]; uint32_t rejected; };

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Packet p;
  if (!info || len != sizeof(p)) return;
  memcpy(&p, data, sizeof(p));
  if (p.magic != MAGIC || p.version != VERSION || !p.boot || p.direction > TURN_RIGHT ||
      p.reason > PATH_FOUND || ((p.clear | p.warning) & 0xe0) || (p.clear & p.warning)) return;
  for (int i = 0; i < BEAMS; ++i) if (p.lifeMs[i] > FRESH_MS) return;
  uint32_t now = millis();
  if (!receiver.accept(info->src_addr, p, now)) { ++receiver.rejected; return; }
  Message msg{};
  msg.rx = {p, now};
  memcpy(msg.source, info->src_addr, 6);
  msg.rejected = receiver.rejected;
  xQueueOverwrite(inbox, &msg);
}

uint8_t obstacleLeds(uint8_t warnings) {
  uint8_t leds = 0;
  if (warnings & FRONT) leds |= 1;
  if (warnings & bitFor(LEFT)) leds |= 2;
  if (warnings & bitFor(RIGHT)) leds |= 4;
  if (warnings & bitFor(DOWN)) leds |= 8;
  return leds;
}

struct Renderer {
  Display decide(uint8_t mode, const Received &rx, bool linked, uint32_t now) const {
    Display d{};
    uint32_t age = now - rx.at;
    if (!linked || age >= LINK_MS) return d;
    const Packet &p = rx.p;
    for (int i = 0; i < BEAMS; ++i)
      if (age + AIR_MARGIN_MS < p.lifeMs[i]) d.valid |= bitFor(i);
    d.warnings = p.warning & d.valid;
    d.reason = p.reason;
    if (!d.valid) { d.view = SENSORS_UNAVAILABLE; return d; }
    if (mode == 1) {
      d.view = OBSTACLES;
      d.leds = obstacleLeds(d.warnings);
      return d;
    }
    uint8_t clear = p.clear & d.valid;
    d.view = PAUSE; d.leds = 8;
    if (d.reason == CALIBRATE) return d;
    if (!(clear & bitFor(DOWN))) { d.reason = CHECK_GROUND; return d; }
    if (p.reason != PATH_FOUND || p.direction == HOLD) return d;
    uint8_t need = p.direction == FORWARD ? FRONT :
                   p.direction == TURN_LEFT ? bitFor(LEFT) : bitFor(RIGHT);
    if ((clear & need) != need) { d.reason = NO_OPEN_BEAM; return d; }
    d.view = DIRECTION;
    d.direction = p.direction;
    d.leds = bitFor(p.direction - 1);
    return d;
  }

  uint8_t pattern(const Display &d, uint32_t elapsed) const {
    if (d.view == LINK_LOST || d.view == SENSORS_UNAVAILABLE) {
      uint32_t phase = elapsed % 1100;
      return phase < 120 || (phase >= 240 && phase < 360) ? ALL_LEDS : 0;
    }
    if (d.view == OBSTACLES) return elapsed % 500 < 300 ? d.leds : 0;
    if (d.view == LAMP_TEST) return bitFor((elapsed / 250) % 4);
    return d.leds;
  }
};

struct Touch {
  uint8_t raw = 0, stable = 0;
  uint32_t changed = 0;
  bool armed = false;
  uint8_t poll(uint8_t pins, uint32_t now) {
    if (pins == 3) armed = false; // Both pressed: wait for full release.
    if (pins != raw) { raw = pins; changed = now; }
    if (now - changed < DEBOUNCE_MS) return 0;
    if (!pins) armed = true;
    if (stable == pins) return 0;
    stable = pins;
    if (!armed || !pins || pins == 3) return 0;
    armed = false;
    return pins;
  }
};

void writeLeds(uint8_t bits) {
  for (int i = 0; i < 4; ++i)
    digitalWrite(LED_PIN[i], (bits & bitFor(i)) ? LED_ACTIVE : !LED_ACTIVE);
}

bool openRadio() {
  if (!WiFi.mode(WIFI_STA) || esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    return false;
  WiFi.setSleep(false);
  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_register_recv_cb(onReceive) == ESP_OK) return true;
  esp_now_deinit();
  return false;
}

void interfaceTask(void *) {
  bool watched = watchThisTask();
  Status s{};
  Touch touch;
  Renderer renderer;
  Message msg{};
  uint32_t since = 0, testStart = 0, prevUs = 0;
  bool testing = false;
  uint8_t lastPins = 255;
  TickType_t next = xTaskGetTickCount();
  for (;;) {
    if (xQueueReceive(inbox, &msg, 0)) {
      s.paired = true; s.seq = msg.rx.p.seq; s.rejected = msg.rejected;
      memcpy(s.source, msg.source, 6);
    }
    uint32_t now = millis(), nowUs = micros();
    if (prevUs) s.maxTickUs = max(s.maxTickUs, nowUs - prevUs);
    prevUs = nowUs;
    uint8_t pins = 0;
    for (int i = 0; i < 2; ++i)
      if (digitalRead(BUTTON[i]) == TOUCH_ACTIVE) pins |= bitFor(i);
    uint8_t selected = touch.poll(pins, now);
    char cmd;
    while (xQueueReceive(uiCommands, &cmd, 0)) {
      if (cmd == 't') { testing = true; testStart = now; }
      else if (cmd == 'x') testing = false;
      else selected = cmd - '0';
    }
    if (selected) { s.mode = selected; ++s.touches; testing = false; }
    Display d = renderer.decide(s.mode, msg.rx, s.paired, now);
    if (testing && now - testStart >= 1000) testing = false;
    if (testing) d.view = LAMP_TEST;
    if (d.view != s.display.view || (d.view == OBSTACLES && !s.display.leds && d.leds))
      since = now;
    uint8_t output = renderer.pattern(d, now - (testing ? testStart : since));
    if (output != lastPins) { writeLeds(output); lastPins = output; }
    s.display = d; s.age = now - msg.rx.at;
    xQueueOverwrite(monitorBox, &s);
    if (watched) esp_task_wdt_reset();
    vTaskDelayUntil(&next, pdMS_TO_TICKS(10));
  }
}

void printHelp() {
  Serial.println("B | Touch GPIO9 / send 1: obstacle mode. Touch GPIO10 / send 2: guidance mode.");
  Serial.println("s = status; h = help; t = 1-second bench lamp test (overrides guidance); x = cancel test.");
  Serial.println("Mode 1: blocked beams blink. Mode 2: forward / turn left / turn right; steady DOWN = HOLD.");
  Serial.println("All four double-flash = unavailable. Send s for the reason.");
}

void printStatus() {
  Status s;
  if (!xQueuePeek(monitorBox, &s, 0)) { Serial.println("B | Starting interface"); return; }
  const char *why[] = {"calibrate floor on A", "ground uncertain", "no confirmed direction", "direction available"};
  const char *dir[] = {"HOLD", "FORWARD", "TURN LEFT then rescan", "TURN RIGHT then rescan"};
  const Display &d = s.display;
  Serial.printf("B | Mode %u | ", s.mode);
  switch (d.view) {
    case LINK_LOST: Serial.print("Waiting for fresh Node A packets"); break;
    case SENSORS_UNAVAILABLE: Serial.print("Connected; all sensor evidence unavailable"); break;
    case PAUSE: Serial.printf("HOLD: %s", why[d.reason]); break;
    case DIRECTION: Serial.print(dir[d.direction]); break;
    case LAMP_TEST: Serial.print("Bench lamp test"); break;
    case OBSTACLES:
      if (!d.warnings) Serial.print("No confirmed obstacle");
      else {
        Serial.print("Obstacle:");
        const char *names[] = {" LEFT", " CENTER", " RIGHT", " UP", " DOWN"};
        for (int i = 0; i < BEAMS; ++i) if (d.warnings & bitFor(i)) Serial.print(names[i]);
      }
      if (d.valid != 31) Serial.print(" | partial sensor coverage");
      break;
  }
  if (s.paired) {
    Serial.printf(" | A %02X:%02X:%02X:%02X:%02X:%02X | seq %lu | age %lu ms | valid 0x%02X",
      s.source[0], s.source[1], s.source[2], s.source[3], s.source[4], s.source[5],
      (unsigned long)s.seq, (unsigned long)s.age, d.valid);
  }
  Serial.printf(" | max UI tick %lu us | skipped RX %lu | B MAC %s\n",
                (unsigned long)s.maxTickUs, (unsigned long)s.rejected, WiFi.macAddress().c_str());
}

void setup() {
  Serial.begin(115200);
  for (uint8_t pin : LED_PIN) { pinMode(pin, OUTPUT); digitalWrite(pin, !LED_ACTIVE); }
  for (uint8_t pin : BUTTON) pinMode(pin, INPUT);
  inbox = xQueueCreate(1, sizeof(Message)); monitorBox = xQueueCreate(1, sizeof(Status));
  uiCommands = xQueueCreate(4, sizeof(char));
  if (!inbox || !monitorBox || !uiCommands) fatal("B | queue allocation failed; restarting");
  startWatchdog();
  if (xTaskCreatePinnedToCore(interfaceTask, "display", 4096, nullptr, 3, nullptr, 1) != pdPASS)
    fatal("B | task allocation failed; restarting");
  printHelp();
}

void loop() {
  static bool radioReady = false, attempted = false;
  static uint32_t tried = 0;
  if (!radioReady && (!attempted || millis() - tried >= 1000)) {
    attempted = true; tried = millis(); radioReady = openRadio();
    if (radioReady) Serial.printf("B | Radio ready | STA MAC %s | channel %u\n", WiFi.macAddress().c_str(), CHANNEL);
    else Serial.println("B | Radio setup failed. Retrying in 1 second.");
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'h' || c == '?') printHelp();
    else if (c == 's') printStatus();
    else if (c == '1' || c == '2' || c == 't' || c == 'x') {
      if (!xQueueSend(uiCommands, &c, 0)) Serial.println("Busy: repeat command.");
    } else if (c > ' ') Serial.println("Unknown command. Send h.");
  }
  static Status previous{};
  static bool first = true;
  Status s;
  if (xQueuePeek(monitorBox, &s, 0) &&
      (first || s.touches != previous.touches || s.display.view != previous.display.view ||
       s.display.leds != previous.display.leds || s.display.reason != previous.display.reason ||
       s.display.warnings != previous.display.warnings)) {
    printStatus(); first = false; previous = s;
  }
  delay(20);
}
