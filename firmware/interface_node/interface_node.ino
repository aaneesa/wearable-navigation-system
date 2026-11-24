
/*
  WEARABLE NAVIGATION SYSTEM
  NODE B - INTERFACE ESP32-S3 N16R8
  Firmware Version: 2.1

  Arduino ESP32 Core 3.x

  LED1 Forward -> GPIO5
  LED2 Left    -> GPIO6
  LED3 Right   -> GPIO7
  LED4 Down    -> GPIO8

  Touch1 Mode 1 -> GPIO9
  Touch2 Mode 2 -> GPIO10

  MODE 1:
    All LEDs flash twice on selection.
    Afterwards LEDs indicate detected obstacles.

  MODE 2:
    LEDs chase from LED1 to LED4 on selection.
    A blinking direction means FORWARD or turn in place and rescan.
    Steady DOWN means all movement directions have confirmed obstructions.

  Sensor order in Node A:
    LEFT=0, CENTER=1, RIGHT=2, UP=3, DOWN=4
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Packet fields and sensor order must match on both boards.
enum SensorId {
  LEFT,
  CENTER,
  RIGHT,
  UP,
  DOWN,
  N
};
enum Direction {
  HOLD,
  FORWARD,
  TURN_LEFT,
  TURN_RIGHT
};
enum Reason {
  CALIBRATE,
  CHECK_GROUND,
  NO_PATH,
  PATH_FOUND,
  WAIT_FOR_ECHO
};

struct __attribute__((packed)) NavPacket {
  uint32_t boot;
  uint32_t seq;
  uint16_t magic;
  uint8_t version;
  uint8_t recommendedLed;
  uint8_t reason;
  uint8_t obstacleMask;
  uint8_t clearMask;       // Confirmed Mode 2 clearance or normal floor.
  uint8_t blockedMask;     // Positive obstacle evidence for Mode 2.
  uint16_t lifeMs[N];
};

static_assert(sizeof(NavPacket) == 26, "Packet layout mismatch");
constexpr uint16_t MAGIC = 0x4E56;
constexpr uint8_t VERSION = 3;
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint16_t FRESH_MS = 450;


// PIN CONFIGURATION
// ==================================================

constexpr uint8_t LED_PINS[4] = {
  5, 6, 7, 8
};

constexpr uint8_t TOUCH1_PIN = 9;
constexpr uint8_t TOUCH2_PIN = 10;

// Change to LOW for active-low touch modules.
constexpr uint8_t TOUCH_ACTIVE_LEVEL = HIGH;

// Assumes GPIO HIGH turns an LED ON.
constexpr uint8_t LED_ON = HIGH;
constexpr uint8_t LED_OFF = LOW;

// ==================================================
constexpr uint8_t LED_FWD   = 1 << 0;
constexpr uint8_t LED_LEFT  = 1 << 1;
constexpr uint8_t LED_RIGHT = 1 << 2;
constexpr uint8_t LED_DOWN  = 1 << 3;

// ==================================================

constexpr uint32_t FEEDBACK_STEP_MS = 100;
constexpr uint32_t DEBOUNCE_MS = 50;
constexpr uint32_t LINK_TIMEOUT_MS = 300;
const uint8_t EXPECTED_NODE_A[6] = {}; // Or enter the station MAC of your belt.

struct Received {
  NavPacket packet;
  uint32_t at;
};

QueueHandle_t inbox;
Received latest{};
bool receivedPacket = false;
uint8_t selectedMode = 1;
uint8_t feedbackMode = 0;
uint32_t feedbackStartMs = 0;
uint8_t feedbackBaseline = 0;
bool lampTest = false;
uint32_t lampStartMs = 0;

enum View {
  LINK_LOST,
  UNKNOWN,
  OBSTACLES,
  GUIDANCE,
  PAUSE
};
struct Display {
  uint8_t freshMask = 0;
  uint8_t direction = HOLD;
  View view = LINK_LOST;
  uint8_t leds = 0;
  uint8_t reason = CALIBRATE;
};

// Receiver owns packet validation on the Wi-Fi callback thread.
// Session, peer and sequence fields are written only by the Wi-Fi callback.
struct Receiver {
  uint8_t pairedAddress[6] = {};
  bool paired = false;
  bool connected = false;
  uint32_t boot = 0;
  uint32_t lastSeq = 0;
  uint32_t lastRx = 0;
  uint32_t candidateBoot = 0;
  uint32_t candidateSeq = 0;
  uint32_t candidateAt = 0;

  bool acceptPacket(const uint8_t *address, const NavPacket &packet, uint32_t now);
  void receive(const esp_now_recv_info_t *info, const uint8_t *data, int len);
};

// Touch and Renderer are used only from the periodic interface task.
struct Touch {
  uint8_t previous = 0;
  bool armed = false;
  uint32_t changedAt = 0;

  void updateTouchInputs();
};

struct Renderer {
  Display chooseDisplay(uint8_t mode, const Received &message, bool connected, uint32_t now);
  void updateLEDs();
};

Receiver receiver;
Touch touch;
Renderer renderer;
struct InterfaceReport {
  Display display;
  uint32_t receivedAt;
  uint32_t sequence;
  uint8_t mode;
  bool received;
};
QueueHandle_t monitorQueue;
QueueHandle_t commands;
void interfaceTask(void *argument);

uint8_t sensorBit(int sensor) {
  return uint8_t(1U << sensor);
}

// ================= ESP-NOW RECEIVE =================
// Receiver state belongs to the Wi-Fi callback only.
bool Receiver::acceptPacket(const uint8_t *address, const NavPacket &packet, uint32_t now) {
  const uint8_t zeroAddress[6] = {};
  if (memcmp(EXPECTED_NODE_A, zeroAddress, 6) && memcmp(EXPECTED_NODE_A, address, 6)) {
    return false;
  }
  if (paired && memcmp(pairedAddress, address, 6)) {
    return false;
  }
  if (!paired) {
    memcpy(pairedAddress, address, 6);
    paired = true;
  }
  // A duplicate may arrive after a newer packet. It cannot renew the link.
  if (connected && boot == packet.boot && int32_t(packet.seq - lastSeq) <= 0) {
    return false;
  }
  if (connected && now - lastRx < LINK_TIMEOUT_MS) {
    if (boot != packet.boot) {
      return false;
    }
  } else {
    if (candidateBoot != packet.boot || now - candidateAt >= LINK_TIMEOUT_MS) {
      candidateBoot = packet.boot;
      candidateSeq = packet.seq;
      candidateAt = now;
      return false;
    }
    if (int32_t(packet.seq - candidateSeq) <= 0) {
      return false;
    }
  }
  candidateBoot = 0;
  connected = true;
  boot = packet.boot;
  lastSeq = packet.seq;
  lastRx = now;
  return true;
}

void Receiver::receive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len != sizeof(NavPacket)) {
    return;
  }
  Received message{};
  memcpy(&message.packet, data, len);
  const NavPacket &packet = message.packet;
  if (packet.magic != MAGIC || packet.version != VERSION || !packet.boot) {
    return;
  }
  if (packet.recommendedLed > TURN_RIGHT || packet.reason > WAIT_FOR_ECHO) {
    return;
  }
  if ((packet.clearMask | packet.obstacleMask | packet.blockedMask) & 0xE0) {
    return;
  }
  if (packet.clearMask & (packet.obstacleMask | packet.blockedMask)) {
    return;
  }
  for (int i = 0; i < N; i++) {
    if (packet.lifeMs[i] > FRESH_MS) {
      return;
    }
  }
  message.at = millis();
  if (acceptPacket(info->src_addr, packet, message.at)) {
    xQueueOverwrite(inbox, &message);
  }
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  receiver.receive(info, data, len);
}

void setLedMask(uint8_t mask) {
  for (int i = 0; i < 4; i++) {
    bool turnOn = mask & (1 << i);

    digitalWrite(
      LED_PINS[i],
      turnOn ? LED_ON : LED_OFF
    );
  }
}

void allLedsOff() {
  setLedMask(0);
}

void startModeFeedback(uint8_t mode) {
  feedbackMode = mode;
  feedbackStartMs = millis();
}

// Returns true while animation is active.
bool showModeFeedback() {
  if (feedbackMode == 0) {
    return false;
  }

  uint32_t elapsed = millis() - feedbackStartMs;

  // MODE 1: all four LEDs flash twice.
  if (feedbackMode == 1) {
    if (elapsed >= 4 * FEEDBACK_STEP_MS) {
      feedbackMode = 0;
      return false;
    }

    bool on =
      ((elapsed / FEEDBACK_STEP_MS) % 2) == 0;

    setLedMask(on ? 0x0F : 0);
    return true;
  }

  // MODE 2: LED1 -> LED2 -> LED3 -> LED4.
  if (feedbackMode == 2) {
    if (elapsed >= 4 * FEEDBACK_STEP_MS) {
      feedbackMode = 0;
      return false;
    }

    uint8_t step = elapsed / FEEDBACK_STEP_MS;

    setLedMask(1 << step);
    return true;
  }

  feedbackMode = 0;
  return false;
}

// ==================================================
// TOUCH INPUT
// ==================================================

bool isTouched(uint8_t pin) {
  return digitalRead(pin) == TOUCH_ACTIVE_LEVEL;
}

uint8_t obstacleToLedMask(uint8_t obstacleMask) {
  uint8_t result = 0;

  // CENTER or UP -> Forward LED1
  if (
    obstacleMask &
    ((1 << CENTER) | (1 << UP))
  ) {
    result |= LED_FWD;
  }

  // LEFT -> LED2
  if (obstacleMask & (1 << LEFT)) {
    result |= LED_LEFT;
  }

  // RIGHT -> LED3
  if (obstacleMask & (1 << RIGHT)) {
    result |= LED_RIGHT;
  }

  // DOWN -> LED4
  if (obstacleMask & (1 << DOWN)) {
    result |= LED_DOWN;
  }

  return result;
}

// ==================================================
// ================= DISPLAY DECISIONS =================
Display Renderer::chooseDisplay(uint8_t mode, const Received &message, bool connected, uint32_t now) {
  Display result;
  uint32_t age = now - message.at;
  if (!connected || age >= LINK_TIMEOUT_MS) {
    return result;
  }
  const NavPacket &packet = message.packet;
  uint8_t fresh = 0;
  for (int i = 0; i < N; i++) {
    if (age + 30 < packet.lifeMs[i]) {
      fresh |= sensorBit(i);
    }
  }
  result.freshMask = fresh;
  result.reason = packet.reason;
  if (!fresh) {
    result.reason = WAIT_FOR_ECHO;
    result.view = UNKNOWN;
    return result;
  }
  if (mode == 1) {
    result.view = OBSTACLES;
    result.leds = obstacleToLedMask(packet.obstacleMask & fresh);
    return result;
  }

  // Missing evidence gets a double flash. Steady DOWN requires fresh
  // positive evidence that forward, left and right are all blocked.
  result.view = UNKNOWN;
  result.reason = packet.reason;
  uint8_t clear = packet.clearMask & fresh;
  uint8_t blocked = packet.blockedMask & fresh;
  bool frontBlocked = blocked & (sensorBit(CENTER) | sensorBit(UP) | sensorBit(DOWN));
  if (packet.reason == NO_PATH && frontBlocked &&
      (blocked & sensorBit(LEFT)) && (blocked & sensorBit(RIGHT))) {
    result.view = PAUSE;
    result.leds = LED_DOWN;
    return result;
  }
  if (packet.reason != PATH_FOUND) {
    if (packet.reason == NO_PATH) result.reason = WAIT_FOR_ECHO;
    return result;
  }
  uint8_t needed = 0;
  switch (packet.recommendedLed) {
    case FORWARD:
      needed = sensorBit(CENTER) | sensorBit(UP) | sensorBit(DOWN);
      break;
    case TURN_LEFT:
      needed = sensorBit(LEFT);
      break;
    case TURN_RIGHT:
      needed = sensorBit(RIGHT);
      break;
    default: return result;
  }
  if ((clear & needed) != needed) {
    result.reason = WAIT_FOR_ECHO;
    return result;
  }
  result.view = GUIDANCE;
  result.direction = packet.recommendedLed;
  result.leds = uint8_t(1U << (packet.recommendedLed - 1));
  return result;
}

void selectMode(uint8_t mode) {
  selectedMode = mode;
  lampTest = false;
  Display current = renderer.chooseDisplay(mode, latest, receivedPacket, millis());
  feedbackBaseline = current.leds;
  startModeFeedback(mode);
  Serial.printf("Mode %u selected\n", mode);
}

void Touch::updateTouchInputs() {
  uint8_t buttons = 0;
  if (isTouched(TOUCH1_PIN)) {
    buttons |= 1;
  }
  if (isTouched(TOUCH2_PIN)) {
    buttons |= 2;
  }
  // Both pads at once: require a full release before accepting a mode.
  if (buttons == 3) {
    armed = false;
  }
  if (buttons != previous) {
    previous = buttons;
    changedAt = millis();
  }
  if (millis() - changedAt < DEBOUNCE_MS) {
    return;
  }
  if (buttons == 0) {
    armed = true;
  } else if (armed && buttons != 3) {
    armed = false;
    selectMode(buttons);
  }
}

void Renderer::updateLEDs() {
  static uint32_t patternStart = 0;
  static int previousView = -1;
  static uint8_t previousMask = 0;
  uint32_t now = millis();
  Display current = renderer.chooseDisplay(selectedMode, latest, receivedPacket, now);
  if (current.view != previousView || (current.leds && previousMask != current.leds)) {
    patternStart = now;
  }
  previousView = current.view;
  previousMask = current.leds;
  uint32_t phase = now - patternStart;

  if (lampTest && now - lampStartMs < 1000) {
    setLedMask(uint8_t(1U << ((now - lampStartMs) / 250)));
    return;
  }
  lampTest = false;
  if (current.view == LINK_LOST || current.view == UNKNOWN) {
    feedbackMode = 0;
    phase %= 1100;
    if (phase < 120 || (phase >= 240 && phase < 360)) {
      setLedMask(15);
    }
    else {
      allLedsOff();
    }
    return;
  }
  // Give a new hazard, changed instruction or HOLD priority over the chase.
  if (current.view == PAUSE || current.leds != feedbackBaseline) {
    feedbackMode = 0;
  }
  if (showModeFeedback()) {
    return;
  }
  if ((current.view == OBSTACLES || current.view == GUIDANCE) && phase % 500 >= 300) {
    allLedsOff();
  }
  else {
    setLedMask(current.leds);
  }
}

void printDebug() {
  InterfaceReport report{};
  if (xQueuePeek(monitorQueue, &report, 0) != pdTRUE) {
    Serial.println("Waiting for display task.");
    return;
  }
  const Display &current = report.display;
  const char *views[] = {"LINK LOST","WAIT / SENSOR UNCERTAIN","OBSTACLES","GUIDANCE","HOLD"};
  const char *reasons[] = {"calibrate floor on A","ground uncertain","all paths blocked","path available","waiting for clear echoes"};
  Serial.printf("B | mode %u | %s | LED mask %u | seq %lu\n", report.mode,
                views[current.view], current.leds, (unsigned long)report.sequence);
  Serial.printf("Fresh sensor mask: 0x%02X\n", current.freshMask);
  if (report.received) {
    Serial.printf("Accepted packet age: %lu ms\n", (unsigned long)(millis() - report.receivedAt));
  }
  if (current.view == PAUSE || current.view == UNKNOWN) {
    Serial.println(reasons[current.reason]);
  }
}

bool startRadio() {
  if (!WiFi.mode(WIFI_STA)) {
    return false;
  }
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    return false;
  }
  WiFi.setSleep(false);
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  if (esp_now_register_recv_cb(onReceive) == ESP_OK) {
    return true;
  }
  esp_now_deinit();
  return false;
}

void setup() {
  Serial.begin(115200);
  inbox = xQueueCreate(1, sizeof(Received));
  monitorQueue = xQueueCreate(1, sizeof(InterfaceReport));
  commands = xQueueCreate(4, sizeof(char));
  if (!inbox || !monitorQueue || !commands) {
    ESP.restart();
  }
  if (xTaskCreatePinnedToCore(
        interfaceTask, "touch-and-leds", 4096, nullptr, 3, nullptr, 1
      ) != pdPASS) {
    ESP.restart();
  }
  Serial.println("Node B: 1/2 select mode; s status; t lamp test; x cancel.");
}

void interfaceTask(void *) {
  for (int i = 0; i < 4; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LED_OFF);
  }
  pinMode(TOUCH1_PIN, INPUT);
  pinMode(TOUCH2_PIN, INPUT);
  TickType_t nextWake = xTaskGetTickCount();
  for (;;) {
    if (xQueueReceive(inbox, &latest, 0) == pdTRUE) {
      receivedPacket = true;
    }
    touch.updateTouchInputs();
    char command;
    while (xQueueReceive(commands, &command, 0) == pdTRUE) {
      if (command == '1' || command == '2') {
        selectMode(command - '0');
      }
      if (command == 't') {
        lampTest = true;
        lampStartMs = millis();
      }
      if (command == 'x') {
        lampTest = false;
      }
    }
    renderer.updateLEDs();
    InterfaceReport report{};
    report.mode = selectedMode;
    report.received = receivedPacket;
    report.receivedAt = latest.at;
    report.sequence = latest.packet.seq;
    report.display = renderer.chooseDisplay(selectedMode, latest, receivedPacket, millis());
    xQueueOverwrite(monitorQueue, &report);
    vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(10));
  }
}

void loop() {
  static bool ready = false;
  static uint32_t lastAttempt = millis() - 1000;
  static uint32_t lastDebug = 0;

  if (!ready && millis() - lastAttempt >= 1000) {
    lastAttempt = millis();
    ready = startRadio();
    if (!ready) {
      Serial.println("Radio setup failed. Retrying.");
    }
  }
  while (Serial.available()) {
    char command = Serial.read();
    if (command == 's') {
      printDebug();
    } else if (command == '1' || command == '2' || command == 't' || command == 'x') {
      if (xQueueSend(commands, &command, 0) != pdTRUE) {
        Serial.println("Busy. Repeat command.");
      }
    } else if (command > ' ') {
      Serial.println("1/2: mode | s: status | t/x: start/stop lamp test");
    }
  }
  if (millis() - lastDebug >= 1000) {
    lastDebug = millis();
    printDebug();
  }
  delay(10);
}
