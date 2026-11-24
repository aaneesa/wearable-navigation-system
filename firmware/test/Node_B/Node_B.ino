
/*
  WEARABLE NAVIGATION SYSTEM
  NODE B - INTERFACE ESP32-S3 N16R8
  Firmware Version: 2.0.0

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
    Afterwards ONLY the recommended direction blinks.
    No recommendation or link loss: all LEDs off.

  Sensor order in Node A:
    LEFT=0, CENTER=1, RIGHT=2, UP=3, DOWN=4
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

// ==================================================
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
// ESP-NOW CONFIGURATION
// ==================================================

// Replace with Node A Wi-Fi station MAC address.
// Broadcast is suitable only for initial testing.
uint8_t PEER_MAC[6] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

constexpr uint8_t WIFI_CHANNEL = 1;

constexpr uint16_t MAGIC = 0x4E56;
constexpr uint8_t VERSION = 2;
constexpr uint8_t FLAG_NO_PATH = 4;

// ==================================================
// TIMING
// ==================================================

constexpr uint32_t MODE_TX_MS = 250;
constexpr uint32_t LINK_TIMEOUT_MS = 600;
constexpr uint32_t DEBOUNCE_MS = 50;
constexpr uint32_t DEBUG_INTERVAL_MS = 1000;

// Each animation step lasts 180 milliseconds.
constexpr uint32_t FEEDBACK_STEP_MS = 180;
constexpr uint32_t PATH_BLINK_MS = 250; // 250 ms ON, 250 ms OFF

// ==================================================
// SENSOR IDS - MUST MATCH NODE A
// ==================================================

enum SensorId {
  LEFT = 0,
  CENTER = 1,
  RIGHT = 2,
  UP = 3,
  DOWN = 4
};

// ==================================================
// LED MASKS
// ==================================================

constexpr uint8_t LED_FWD   = 1 << 0;
constexpr uint8_t LED_LEFT  = 1 << 1;
constexpr uint8_t LED_RIGHT = 1 << 2;
constexpr uint8_t LED_DOWN  = 1 << 3;

// ==================================================
// PACKET STRUCTURES - MUST MATCH NODE A
// ==================================================

struct __attribute__((packed)) ModePacket {
  uint16_t magic;
  uint8_t version;
  uint8_t mode;
  uint32_t seq;
};

struct __attribute__((packed)) NavPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t mode;
  uint32_t seq;

  uint8_t obstacleMask;
  uint8_t validMask;
  uint8_t clearLedMask;
  uint8_t recommendedLed;

  uint8_t personalMask;
  int16_t distanceCm[5];
  uint8_t flags;
};

// ==================================================
// GLOBAL VARIABLES
// ==================================================

static_assert(sizeof(ModePacket)==8, "Mode packet layout");
static_assert(sizeof(NavPacket)==24, "Navigation packet layout");

void getLatestPacket(NavPacket &packet, uint32_t &rxTime, bool &hasPacket);
uint8_t recommendedMask(const NavPacket& p);

uint8_t selectedMode = 1;
uint8_t activePathMask = 0;
uint32_t pathBlinkStartMs = 0;

uint32_t modeSeq = 0;
uint32_t lastModeTxMs = 0;
uint32_t lastDebugMs = 0;

// Latest navigation packet
NavPacket latestPacket = {};
uint32_t lastRxMs = 0;
bool receivedPacket = false;

// Protect shared data between callback and loop.
portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;

// Touch debounce
bool lastRawTouch1 = false;
bool lastRawTouch2 = false;

bool stableTouch1 = false;
bool stableTouch2 = false;

uint32_t touch1ChangeMs = 0;
uint32_t touch2ChangeMs = 0;

// Mode-selection LED animation
uint8_t feedbackMode = 0;
uint32_t feedbackStartMs = 0;

// ==================================================
// ESP-NOW RECEIVE CALLBACK
// ==================================================

void onReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  if (len != (int)sizeof(NavPacket)) {
    return;
  }

  if (PEER_MAC[0] != 0xFF && memcmp(info->src_addr, PEER_MAC, 6)!=0) return;
  NavPacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.magic != MAGIC) return;
  if (packet.version != VERSION) return;
  if (packet.mode != 1 && packet.mode != 2) return;

  portENTER_CRITICAL(&packetMux);

  latestPacket = packet;
  lastRxMs = millis();
  receivedPacket = true;

  portEXIT_CRITICAL(&packetMux);
}

// ==================================================
// LED FUNCTIONS
// ==================================================

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

// Startup test: one LED at a time.
void startupLedTest() {
  Serial.println("Starting LED test...");

  for (int i = 0; i < 4; i++) {
    allLedsOff();
    digitalWrite(LED_PINS[i], LED_ON);
    delay(250);
  }

  allLedsOff();

  Serial.println("LED test complete");
}

// ==================================================
// MODE SELECTION FEEDBACK
// ==================================================

void startModeFeedback(uint8_t mode) {
  activePathMask = 0;
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

void updateTouchInputs() {
  uint32_t now = millis();

  bool raw1 = isTouched(TOUCH1_PIN);
  bool raw2 = isTouched(TOUCH2_PIN);

  // Detect changes in raw touch signals.
  if (raw1 != lastRawTouch1) {
    lastRawTouch1 = raw1;
    touch1ChangeMs = now;
  }

  if (raw2 != lastRawTouch2) {
    lastRawTouch2 = raw2;
    touch2ChangeMs = now;
  }

  // Touch 1: select Mode 1.
  if (
    raw1 != stableTouch1 &&
    now - touch1ChangeMs >= DEBOUNCE_MS
  ) {
    stableTouch1 = raw1;

    if (stableTouch1 && selectedMode != 1) {
      selectedMode = 1;

      startModeFeedback(1);

      Serial.println();
      Serial.println("MODE 1 SELECTED");
      Serial.println("Obstacle detection active");
    }
  }

  // Touch 2: select Mode 2.
  if (
    raw2 != stableTouch2 &&
    now - touch2ChangeMs >= DEBOUNCE_MS
  ) {
    stableTouch2 = raw2;

    if (stableTouch2 && selectedMode != 2) {
      selectedMode = 2;

      startModeFeedback(2);

      Serial.println();
      Serial.println("MODE 2 SELECTED");
      Serial.println("Single recommended direction blinking active");
    }
  }
}

// ==================================================
// SEND MODE TO NODE A
// ==================================================

void sendMode() {
  ModePacket packet = {};

  packet.magic = MAGIC;
  packet.version = VERSION;
  packet.mode = selectedMode;
  packet.seq = ++modeSeq;

  esp_err_t result = esp_now_send(
    PEER_MAC,
    reinterpret_cast<uint8_t*>(&packet),
    sizeof(packet)
  );

  if (result != ESP_OK) {
    Serial.print("ESP-NOW mode send error: ");
    Serial.println((int)result);
  }
}

// ==================================================
// OBSTACLE MASK -> LED MASK
// ==================================================

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
// GET LATEST PACKET SAFELY
// ==================================================

void getLatestPacket(
  NavPacket &packet,
  uint32_t &rxTime,
  bool &hasPacket
) {
  portENTER_CRITICAL(&packetMux);

  packet = latestPacket;
  rxTime = lastRxMs;
  hasPacket = receivedPacket;

  portEXIT_CRITICAL(&packetMux);
}

// ==================================================
// LED DISPLAY CONTROL
// ==================================================

uint8_t recommendedMask(const NavPacket& p) {
  if ((p.flags & FLAG_NO_PATH) || !(p.clearLedMask & LED_DOWN)) return 0;
  uint8_t mask=0;
  switch(p.recommendedLed) {
    case 1: mask=LED_FWD; break;
    case 2: mask=LED_LEFT; break;
    case 3: mask=LED_RIGHT; break;
    default: return 0;
  }
  return (p.clearLedMask & mask) ? mask : 0;
}

void updateLEDs() {
  // Original mode-selection animation, not a navigation instruction.
  if (showModeFeedback()) return;
  NavPacket packet; uint32_t rxTime; bool hasPacket;
  getLatestPacket(packet, rxTime, hasPacket);
  uint32_t now=millis();
  bool linkOK=hasPacket && (now-rxTime<LINK_TIMEOUT_MS);
  if (!linkOK) {
    activePathMask=0;
    if(selectedMode==1) { // original Mode 1 link-loss alert
      setLedMask(((now/300)%2)==0 ? 0x0F : 0);
    } else allLedsOff(); // never display a false direction in Mode 2
    return;
  }
  if(packet.mode!=selectedMode) {
    activePathMask=0; allLedsOff(); return;
  }
  if(selectedMode==1) {
    activePathMask=0;
    setLedMask(obstacleToLedMask(packet.obstacleMask));
    return;
  }
  // Only one direction is allowed. Reset phase on a changed recommendation.
  uint8_t mask=recommendedMask(packet);
  if(mask!=activePathMask) {activePathMask=mask;pathBlinkStartMs=now;}
  if(!mask){allLedsOff();return;}
  bool on=((now-pathBlinkStartMs)/PATH_BLINK_MS)%2==0;
  setLedMask(on?mask:0);
}

// ==================================================
// SERIAL MONITOR DEBUG
// ==================================================

void printDebug() {
  NavPacket packet;
  uint32_t rxTime;
  bool hasPacket;

  getLatestPacket(packet, rxTime, hasPacket);

  bool linkOK =
    hasPacket &&
    (millis() - rxTime < LINK_TIMEOUT_MS);

  Serial.println();
  Serial.println("========== NODE B DEBUG ==========");

  Serial.print("Selected mode: ");
  Serial.println(selectedMode);

  Serial.print("ESP-NOW link: ");
  Serial.println(linkOK ? "CONNECTED" : "DISCONNECTED");

  if (linkOK) {
    Serial.print("Node A packet mode: ");
    Serial.println(packet.mode);

    Serial.print("Packet sequence: ");
    Serial.println(packet.seq);

    Serial.print("Obstacle mask: 0x");
    Serial.println(packet.obstacleMask, HEX);

    Serial.print("Valid mask: 0x");
    Serial.println(packet.validMask, HEX);

    Serial.print("Personal mask: 0x");
    Serial.println(packet.personalMask, HEX);

    Serial.print("Clear LED mask: 0x");
    Serial.println(packet.clearLedMask, HEX);

    Serial.print("Recommended direction: ");

    switch (packet.recommendedLed) {
      case 1:
        Serial.println("FORWARD");
        break;

      case 2:
        Serial.println("TURN LEFT");
        break;

      case 3:
        Serial.println("TURN RIGHT");
        break;

      default:
        Serial.println("NONE / STOP");
        break;
    }
  }

  Serial.println("==================================");
}

// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("WEARABLE NAVIGATION SYSTEM");
  Serial.println("Node B - Interface ESP32-S3");
  Serial.println("Firmware Version 2.0.0");

  // Configure LEDs.
  for (int i = 0; i < 4; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LED_OFF);
  }

  // Configure digital touch modules.
  pinMode(TOUCH1_PIN, INPUT);
  pinMode(TOUCH2_PIN, INPUT);

  // Initialize touch debounce state.
  lastRawTouch1 = isTouched(TOUCH1_PIN);
  lastRawTouch2 = isTouched(TOUCH2_PIN);

  stableTouch1 = lastRawTouch1;
  stableTouch2 = lastRawTouch2;

  // Verify LED wiring.
  startupLedTest();

  // Initialize Wi-Fi.
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(WIFI_CHANNEL);

  Serial.print("Interface MAC: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW.
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW initialization failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};

  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERROR: ESP-NOW peer registration failed");
  }
  else {
    Serial.println("ESP-NOW initialized successfully");
  }

  Serial.println("Node B ready");
}

// ==================================================
// LOOP
// ==================================================

void loop() {
  // Check touch sensors.
  updateTouchInputs();

  uint32_t now = millis();

  // Send mode selection to Node A.
  if (now - lastModeTxMs >= MODE_TX_MS) {
    lastModeTxMs = now;
    sendMode();
  }

  // Control LEDs.
  updateLEDs();

  // Serial debugging.
  if (now - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = now;
    printDebug();
  }
}
