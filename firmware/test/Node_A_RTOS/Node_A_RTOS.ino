// Wearable navigation | Node A: sensing and guidance | ESP32-S3, Arduino-ESP32 3.x
// Serial Monitor: 115200 baud, Newline. Send h for controls.
// Standard 5 V HC-SR04 modules need 3.3 V level shifting on every ECHO input.
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_random.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>

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

constexpr uint8_t TRIG[BEAMS] = {4, 6, 14, 10, 12};
constexpr uint8_t ECHO[BEAMS] = {5, 7, 15, 11, 13};
constexpr uint8_t PEER[6] = {255, 255, 255, 255, 255, 255}; // Or enter B's STA MAC.
constexpr uint16_t SLOT_MS = 55, HEARTBEAT_MS = 100;
constexpr uint32_t ECHO_US = 16000;
constexpr int ENTER_CM = 90, EXIT_CM = 110, TURN_MARGIN_CM = 15;
constexpr int FLOOR_BAND_CM = 18, CAL_SAMPLES = 8;
const char *const NAMES[] = {"LEFT", "CENTER", "RIGHT", "UP", "DOWN"};
const char *const ACTION[] = {"HOLD", "FORWARD", "TURN LEFT", "TURN RIGHT"};

enum EchoError : int16_t { NO_ECHO = -1, TOO_NEAR = -2, TOO_FAR = -3, STUCK_HIGH = -4 };
struct Reading { uint32_t seq, at; uint8_t id; int16_t cm; };
struct Beam {
  int16_t cm = NO_ECHO;
  uint32_t at = 0;
  uint8_t farCount = 0;
  bool clear = false, warn = false, pendingNear = false;
};
struct Frame {
  Packet p{};
  uint32_t at = 0, maxSlotUs = 0, missedSlots = 0, timeouts = 0;
  int16_t cm[BEAMS]{};
  uint16_t floorCm = 0;
  uint8_t calCount = 0, calState = 0, leftZone = 0, rightZone = 0;
};
enum CalState : uint8_t { NEED_FLOOR, COLLECTING, READY, UNSTABLE };
struct Command { char key; uint8_t cm; };

QueueHandle_t samples, outbox, monitorBox, commands, savedFloor;
SemaphoreHandle_t echoDone;
Preferences prefs;
uint32_t bootId;
bool prefsOK;

// Pure guidance logic. Only the navigation task owns these fields.
struct Navigation {
  Beam beam[BEAMS]{};
  int zone[BEAMS] = {20, 0, 20, 0, 0};  // Side bands only, measured from each transducer.
  int floorCm = 0, count = 0, low = 0, high = 0, total = 0;
  CalState calibration = NEED_FLOOR;
  Direction lastChoice = HOLD;

  void beginCalibration() {
    calibration = COLLECTING;
    floorCm = count = total = 0;
    low = 251;
    high = 0;
    beam[DOWN] = Beam{};
  }

  void collectFloor(int cm) {
    // Wear the belt, stand still, and point DOWN at an unobstructed level floor.
    if (cm < 35 || cm > 230) { calibration = UNSTABLE; return; }
    low = min(low, cm);
    high = max(high, cm);
    if (high - low > 10) { calibration = UNSTABLE; return; }
    total += cm;
    if (++count == CAL_SAMPLES) {
      floorCm = (total + count / 2) / count;
      calibration = READY;
    }
  }

  void update(const Reading &r) {
    Beam &b = beam[r.id];
    if (r.at - b.at >= FRESH_MS) b = Beam{};
    bool wasClear = b.clear;
    bool repeated = b.pendingNear || b.warn;
    b.at = r.at;
    b.cm = r.cm;
    if (r.id == DOWN && calibration == COLLECTING) {
      collectFloor(r.cm);
      b.clear = b.warn = false;
      b.farCount = 0;
      return;
    }
    if (r.cm < 0 || (r.id == DOWN && calibration != READY)) {
      b = Beam{};
      b.cm = r.cm;
      b.at = r.at;
      return;
    }

    bool near = r.cm < ENTER_CM;
    bool far = r.cm > EXIT_CM;
    if (r.id == DOWN) {
      near = r.cm < floorCm - FLOOR_BAND_CM;
      far = abs(r.cm - floorCm) <= FLOOR_BAND_CM;
      if (!near && !far) {  // Long return: ground uncertain.
        b.clear = b.warn = b.pendingNear = false;
        b.farCount = 0;
        return;
      }
    }
    if (near) {
      bool limbZone = (zone[r.id] > 0 && r.cm <= zone[r.id]) || r.id == DOWN;
      const Beam &center = beam[CENTER];
      bool corroborated = center.warn && r.at - center.at < FRESH_MS;
      b.clear = false;  // First close echo always withdraws movement guidance.
      b.warn = !limbZone || !wasClear || repeated || corroborated;
      b.pendingNear = !b.warn;
      b.farCount = 0;
    } else if (far) {
      b.pendingNear = false;
      if (b.farCount < 2) ++b.farCount;
      if (b.farCount == 2) { b.clear = true; b.warn = false; }
    } else {
      // 90..110 cm retains an established state; it cannot establish clearance.
      b.farCount = 0;
      b.pendingNear = false;
    }
  }

  Direction choose(uint8_t clear) const {
    if (!(clear & bitFor(DOWN))) return HOLD;
    if ((clear & FRONT) == FRONT) return FORWARD;
    bool left = clear & bitFor(LEFT), right = clear & bitFor(RIGHT);
    if (left && right) {
      int delta = beam[LEFT].cm - beam[RIGHT].cm;
      if (abs(delta) <= TURN_MARGIN_CM &&
          (lastChoice == TURN_LEFT || lastChoice == TURN_RIGHT)) return lastChoice;
      return delta >= 0 ? TURN_LEFT : TURN_RIGHT;
    }
    if (left) return TURN_LEFT;
    if (right) return TURN_RIGHT;
    return HOLD;
  }

  Frame snapshot(uint32_t now) {
    Frame f{};
    f.at = now;
    f.floorCm = floorCm;
    f.calState = calibration;
    f.calCount = count;
    f.leftZone = zone[LEFT];
    f.rightZone = zone[RIGHT];
    for (int i = 0; i < BEAMS; ++i) {
      const Beam &b = beam[i];
      f.cm[i] = b.cm;
      uint32_t age = now - b.at;
      if (b.cm < 0 || age >= FRESH_MS) continue;
      f.p.lifeMs[i] = FRESH_MS - age;
      if (b.clear) f.p.clear |= bitFor(i);
      if (b.warn) f.p.warning |= bitFor(i);
    }
    lastChoice = choose(f.p.clear);
    f.p.direction = lastChoice;
    if (calibration != READY) f.p.reason = CALIBRATE;
    else if (!(f.p.clear & bitFor(DOWN))) f.p.reason = CHECK_GROUND;
    else f.p.reason = lastChoice == HOLD ? NO_OPEN_BEAM : PATH_FOUND;
    return f;
  }
};

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

// Echo capture. The ISR only timestamps the currently armed input and wakes its owner.
portMUX_TYPE captureLock = portMUX_INITIALIZER_UNLOCKED;
volatile int armedPin = -1;
volatile bool risingSeen = false, finished = false;
volatile uint32_t armedUs, riseUs, widthUs;
struct ScanStats { uint32_t maxSlotUs, missedSlots, timeouts; };
QueueHandle_t scanStats;

void ARDUINO_ISR_ATTR echoEdge(void *arg) {
  int pin = int(uintptr_t(arg));
  uint32_t now = micros();
  bool wake = false;
  portENTER_CRITICAL_ISR(&captureLock);
  if (pin == armedPin && now - armedUs <= ECHO_US) {
    if (gpio_get_level(gpio_num_t(pin))) {
      if (!risingSeen) { riseUs = now; risingSeen = true; }
    } else if (risingSeen) {
      widthUs = now - riseUs;
      armedPin = -1;
      finished = wake = true;
    }
  }
  portEXIT_CRITICAL_ISR(&captureLock);
  if (wake) {
    BaseType_t higher = pdFALSE;
    xSemaphoreGiveFromISR(echoDone, &higher);
    if (higher) portYIELD_FROM_ISR();
  }
}

int measure(uint8_t id) {
  if (digitalRead(ECHO[id])) return STUCK_HIGH;
  xSemaphoreTake(echoDone, 0);
  portENTER_CRITICAL(&captureLock);
  risingSeen = finished = false;
  armedUs = micros();
  armedPin = ECHO[id];
  portEXIT_CRITICAL(&captureLock);
  digitalWrite(TRIG[id], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG[id], LOW);
  xSemaphoreTake(echoDone, pdMS_TO_TICKS(16) + 1);
  portENTER_CRITICAL(&captureLock);
  armedPin = -1;
  bool ok = finished;
  uint32_t us = widthUs;
  portEXIT_CRITICAL(&captureLock);
  if (!ok) return NO_ECHO;
  if (us < 175) return TOO_NEAR;
  if (us > 14577) return TOO_FAR;
  return int((us * 343U + 10000U) / 20000U);
}

void sensorTask(void *) {
  for (int i = 0; i < BEAMS; ++i) {
    attachInterruptArg(ECHO[i], echoEdge, (void *)uintptr_t(ECHO[i]), CHANGE);
  }
  bool watched = watchThisTask();
  uint32_t seq = 0, previousUs = 0;
  ScanStats stats{};
  uint8_t id = LEFT;
  for (;;) {
    TickType_t start = xTaskGetTickCount();
    uint32_t nowUs = micros();
    if (seq) {
      uint32_t gap = nowUs - previousUs;
      stats.maxSlotUs = max(stats.maxSlotUs, gap);
      if (gap > (SLOT_MS + 2U) * 1000U) ++stats.missedSlots;
    }
    previousUs = nowUs;
    Reading r{++seq, uint32_t(millis()), id, int16_t(measure(id))};
    if (r.cm == NO_ECHO) ++stats.timeouts;
    if (xQueueSend(samples, &r, 0) != pdTRUE) {
      xQueueReset(samples);  // Sequence gap clears accumulated evidence below.
      xQueueSend(samples, &r, 0);
    }
    xQueueOverwrite(scanStats, &stats);
    if (watched) esp_task_wdt_reset();
    id = (id + 1) % BEAMS;
    vTaskDelayUntil(&start, pdMS_TO_TICKS(SLOT_MS)); // Anchor to this trigger; no catch-up burst.
  }
}

int initialFloor;
void navigationTask(void *) {
  Navigation nav;
  if (initialFloor) { nav.floorCm = initialFloor; nav.calibration = READY; }
  bool watched = watchThisTask();
  uint32_t seq = 0;
  for (;;) {
    Reading r;
    if (!xQueueReceive(samples, &r, pdMS_TO_TICKS(100))) continue;
    Command cmd;
    while (xQueueReceive(commands, &cmd, 0)) {
      if (cmd.key == 'c') nav.beginCalibration();
      else nav.zone[cmd.key == 'l' ? LEFT : RIGHT] = cmd.cm;
    }
    if (r.seq != seq + 1 || millis() - r.at >= FRESH_MS) {
      for (Beam &b : nav.beam) b = Beam{};
      if (nav.calibration == COLLECTING) nav.calibration = UNSTABLE;
    }
    seq = r.seq;
    bool collecting = nav.calibration == COLLECTING;
    if (millis() - r.at < FRESH_MS) nav.update(r);
    if (collecting && nav.calibration == READY) xQueueOverwrite(savedFloor, &nav.floorCm);
    Frame f = nav.snapshot(millis());
    ScanStats stats{};
    xQueuePeek(scanStats, &stats, 0);
    f.maxSlotUs = stats.maxSlotUs;
    f.missedSlots = stats.missedSlots;
    f.timeouts = stats.timeouts;
    xQueueOverwrite(outbox, &f);
    xQueueOverwrite(monitorBox, &f);
    if (watched) esp_task_wdt_reset();
  }
}

// Wi-Fi callbacks run in task context. A length-one queue holds send completion.
QueueHandle_t sentResult, radioStatus;
struct RadioStatus { bool ready; uint32_t sent, failed; };
// Core 3.x releases use different types for the unused send-info argument.
struct SendCompletion {
  template <typename Info>
  static void record(const Info *, esp_now_send_status_t status) {
    xQueueOverwrite(sentResult, &status);
  }
};

bool openRadio() {
  if (!WiFi.mode(WIFI_STA) || esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    return false;
  WiFi.setSleep(false);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER, 6);
  peer.channel = CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_register_send_cb(SendCompletion::record) == ESP_OK && esp_now_add_peer(&peer) == ESP_OK)
    return true;
  esp_now_deinit();
  return false;
}

void radioTask(void *) {
  bool watched = watchThisTask();
  Frame latest{};
  RadioStatus status{};
  uint32_t seq = 0, lastTx = 0;
  Packet previous{};
  uint8_t failures = 0;
  for (;;) {
    if (watched) esp_task_wdt_reset();
    if (!status.ready) {
      xQueueReset(sentResult);
      status.ready = openRadio();
      xQueueOverwrite(radioStatus, &status);
      if (!status.ready) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }
    }
    Frame fresh;
    if (xQueueReceive(outbox, &fresh, pdMS_TO_TICKS(20))) latest = fresh;
    uint32_t now = millis();
    Packet p = latest.p;
    bool changed = p.direction != previous.direction || p.reason != previous.reason ||
                   p.warning != previous.warning || p.clear != previous.clear;
    if (seq && !changed && now - lastTx < HEARTBEAT_MS) continue;
    uint32_t age = now - latest.at;
    for (int i = 0; i < BEAMS; ++i)
      p.lifeMs[i] = age < p.lifeMs[i] ? p.lifeMs[i] - age : 0;
    p.magic = MAGIC; p.version = VERSION; p.boot = bootId; p.seq = ++seq;
    previous = p;
    lastTx = now;
    esp_now_send_status_t result;
    bool submitted = esp_now_send(PEER, (const uint8_t *)&p, sizeof(p)) == ESP_OK;
    bool completed = submitted && xQueueReceive(sentResult, &result, pdMS_TO_TICKS(80));
    if (completed && result == ESP_NOW_SEND_SUCCESS) {
      ++status.sent;
      failures = 0;
    } else {
      ++status.failed;
      if ((submitted && !completed) || ++failures >= 4) {
        esp_now_deinit();
        status.ready = false;
        failures = 0;
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    xQueueOverwrite(radioStatus, &status);
  }
}

void printHelp() {
  Serial.println("A | c = calibrate floor; s = status; v = live distances on/off; h = help");
  Serial.println("l 20 / r 20 = side limb zone in cm (0 disables; 0..40); resets at reboot.");
  Serial.println("For c: wear belt, stand still on level ground, keep legs clear of DOWN.");
}

void printStatus() {
  Frame f;
  if (!xQueuePeek(monitorBox, &f, 0)) { Serial.println("A | Waiting for first scan"); return; }
  const char *cal[] = {"send c on level floor", "keep still", "ready", "unstable: reposition and send c"};
  const char *why[] = {"floor calibration required", "ground uncertain", "no confirmed direction", "direction available"};
  Serial.printf("A | %s | %s | floor %u cm (%s %u/%d) | side zones %u/%u cm\n",
                ACTION[f.p.direction], why[f.p.reason], f.floorCm, cal[f.calState],
                f.calCount, CAL_SAMPLES, f.leftZone, f.rightZone);
  for (int i = 0; i < BEAMS; ++i) {
    const char *state = f.cm[i] == NO_ECHO ? "no echo" : f.cm[i] == STUCK_HIGH ? "stuck high" :
                        f.cm[i] == TOO_NEAR ? "below range" : f.cm[i] == TOO_FAR ? "above range" :
                        f.p.warning & bitFor(i) ? "obstacle" : f.p.clear & bitFor(i) ? "clear" : "unconfirmed";
    Serial.printf("  %s %d cm | %s\n", NAMES[i], f.cm[i], state);
  }
  RadioStatus rs{};
  xQueuePeek(radioStatus, &rs, 0);
  Serial.printf("  STA MAC %s | channel %u\n", WiFi.macAddress().c_str(), CHANNEL);
  Serial.printf("  radio %s | send completions %lu | failures %lu | echo timeouts %lu | max slot %lu us | late slots %lu\n",
                rs.ready ? "ready" : "retrying", (unsigned long)rs.sent, (unsigned long)rs.failed,
                (unsigned long)f.timeouts, (unsigned long)f.maxSlotUs, (unsigned long)f.missedSlots);
}

bool live = false;
void handleLine(const char *line) {
  if (!strcmp(line, "h") || !strcmp(line, "?")) { printHelp(); return; }
  if (!strcmp(line, "s")) { printStatus(); return; }
  if (!strcmp(line, "v")) { live = !live; Serial.println(live ? "Live ON" : "Live OFF"); return; }
  Command cmd{};
  if (!strcmp(line, "c")) cmd.key = 'c';
  else {
    int cm;
    char extra;
    if (sscanf(line, "%c %d %c", &cmd.key, &cm, &extra) != 2 ||
        (cmd.key != 'l' && cmd.key != 'r') || cm < 0 || cm > 40) {
      Serial.println("Unknown command or value. Send h."); return;
    }
    cmd.cm = cm;
  }
  if (xQueueSend(commands, &cmd, 0) != pdTRUE) Serial.println("Busy: repeat command.");
  else if (cmd.key == 'c') Serial.println("Collecting floor echoes. Keep still for about 3 seconds.");
  else Serial.printf("%s limb band requested: %u cm\n", cmd.key == 'l' ? "LEFT" : "RIGHT", cmd.cm);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < BEAMS; ++i) {
    pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW); pinMode(ECHO[i], INPUT);
  }
  prefsOK = prefs.begin("belt-nav-v2", false);
  initialFloor = prefsOK ? prefs.getUShort("floor", 0) : 0;
  if (initialFloor < 35 || initialFloor > 230) initialFloor = 0;
  WiFi.mode(WIFI_STA);
  Serial.printf("Node A | STA MAC %s | channel %u | stored floor %d cm\n",
                WiFi.macAddress().c_str(), CHANNEL, initialFloor);
  printHelp();
  bootId = esp_random() | 1U;
  samples = xQueueCreate(8, sizeof(Reading));
  outbox = xQueueCreate(1, sizeof(Frame)); monitorBox = xQueueCreate(1, sizeof(Frame));
  scanStats = xQueueCreate(1, sizeof(ScanStats)); radioStatus = xQueueCreate(1, sizeof(RadioStatus));
  commands = xQueueCreate(4, sizeof(Command)); savedFloor = xQueueCreate(1, sizeof(int));
  sentResult = xQueueCreate(1, sizeof(esp_now_send_status_t));
  echoDone = xSemaphoreCreateBinary();
  if (!samples || !outbox || !monitorBox || !scanStats || !commands || !savedFloor ||
      !radioStatus || !sentResult || !echoDone) fatal("A | queue allocation failed; restarting");
  startWatchdog();
  if (xTaskCreatePinnedToCore(sensorTask, "echo", 4096, nullptr, 4, nullptr, 1) != pdPASS ||
      xTaskCreatePinnedToCore(navigationTask, "guide", 4096, nullptr, 3, nullptr, 1) != pdPASS ||
      xTaskCreatePinnedToCore(radioTask, "radio", 6144, nullptr, 2, nullptr, 0) != pdPASS)
    fatal("A | task allocation failed; restarting");
}

void loop() {
  static char line[24];
  static size_t used = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (overflow) Serial.println("Command too long. Send h.");
      else if (used) { line[used] = 0; handleLine(line); }
      used = 0; overflow = false;
    } else if (used < sizeof(line) - 1) line[used++] = c;
    else overflow = true;
  }
  int floor;
  if (xQueueReceive(savedFloor, &floor, 0)) {
    bool saved = prefsOK && prefs.putUShort("floor", floor) == sizeof(uint16_t);
    Serial.printf("Calibration complete: %d cm. %s\n", floor, saved ? "Saved for next boot." : "Active this boot only.");
  }
  static uint8_t lastCal = 255;
  Frame f{};
  if (xQueuePeek(monitorBox, &f, 0) && f.calState != lastCal) {
    if (f.calState == NEED_FLOOR) Serial.println("Mode 2 HOLD | Fit belt and send c on level floor.");
    if (f.calState == UNSTABLE) Serial.println("Calibration failed | Check DOWN, stand still, then send c again.");
    lastCal = f.calState;
  }
  static uint8_t shownCount = 255;
  if (f.calState == COLLECTING && f.calCount != shownCount) {
    Serial.printf("Floor calibration: %u/%d stable echoes\n", f.calCount, CAL_SAMPLES);
    shownCount = f.calCount;
  } else if (f.calState != COLLECTING) shownCount = 255;
  static uint32_t lastPrint = 0;
  if (live && millis() - lastPrint >= 500) { lastPrint = millis(); printStatus(); }
  delay(10);
}
