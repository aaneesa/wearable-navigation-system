/*
  WEARABLE NAVIGATION SYSTEM
  NODE A - BELT WITH FIVE ULTRASONIC SENSORS
  Board: ESP32-S3, Arduino-ESP32 3.x

  Trace one reading:
    sensorTask -> measure -> echoEdge -> ordered reading queue
    navigationTask -> Navigation::processReading -> chooseDirection
    radioTask -> sendNavigation -> Node B on channel 1

  Serial Monitor: 115200 baud, Newline.
    c       Stand still on level ground and calibrate DOWN.
    s       Print the last decision and all five distances.
    v       Toggle continuous sensor status.
    l 15    Set the LEFT personal-space band to 15 cm.
    r 15    Set the RIGHT band. Zero disables a side band.

  Each ultrasonic module takes 3.3 V and shares this board's GND.
  GPIO order below follows the wearer's left, center, right, up and down.
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
  PATH_FOUND
};

struct __attribute__((packed)) NavPacket {
  uint32_t boot;
  uint32_t seq;
  uint16_t magic;
  uint8_t version;
  uint8_t recommendedLed;
  uint8_t reason;
  uint8_t obstacleMask;
  uint8_t clearMask;
  uint16_t lifeMs[N];
};

static_assert(sizeof(NavPacket) == 25, "Packet layout mismatch");
constexpr uint16_t MAGIC = 0x4E56;
constexpr uint8_t VERSION = 2;
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint16_t FRESH_MS = 450;


#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_random.h>
#include <freertos/semphr.h>

// ================= CONFIGURATION =================
const uint8_t TRIG_PINS[N] = {4, 6, 14, 10, 12};
const uint8_t ECHO_PINS[N] = {5, 7, 15, 11, 13};
const uint8_t PEER_MAC[6] = {255,255,255,255,255,255};
const char *const SENSOR_NAMES[N] = {"LEFT","CENTER","RIGHT","UP","DOWN"};
const int ENTER_CM = 90;
const int EXIT_CM = 110;
const int SENSOR_GAP_MS = 55;

struct Reading {
  uint32_t seq;
  uint32_t at;
  uint8_t sensor;
  int cm;
};

struct Beam {
  int cm = -1;
  uint32_t at = 0;
  int clearCount = 0;
  bool clear = false;
  bool obstacle = false;
};

struct Snapshot {
  NavPacket packet;
  uint32_t at;
};

// One navigation task owns all mutable beam and floor state.
struct Navigation {
  void startFloorCalibration();
  void collectFloor(int distanceCm);
  void processReading(const Reading &reading);
  uint8_t chooseDirection(uint8_t clearMask);
  NavPacket makePacket(uint32_t nowMs);

  Beam beam[N];
  int personalBand[N] = {20,0,20,0,0};
  int floorCm = 0;
  int floorCount = 0;
  int floorSum = 0;
  int floorLow = 0;
  int floorHigh = 0;
  bool calibrating = false;


  uint8_t lastDirection = HOLD;
};

Navigation navigation;
bool verbose = false;
bool storageReady = false;
Preferences storage;
uint32_t bootId;

struct Command {
  char key;
  int value;
};

struct DebugSnapshot {
  NavPacket packet;
  int distances[N];
  int floorCm;
  int leftZoneCm;
  int rightZoneCm;
  uint32_t at;
};

QueueHandle_t debugQueue;
QueueHandle_t commandQueue;
QueueHandle_t floorSaveQueue;
void navigationTask(void *argument);

QueueHandle_t readings;
QueueHandle_t outgoing;
SemaphoreHandle_t echoDone;
portMUX_TYPE echoLock = portMUX_INITIALIZER_UNLOCKED;
volatile int activePin = -1;
volatile bool risingSeen = false;
volatile uint32_t riseUs;
volatile uint32_t armedUs;
volatile uint32_t widthUs;

uint8_t sensorBit(int sensor) {
  return uint8_t(1U << sensor);
}

// ================= ULTRASONIC READING =================
// Interrupt context: only the armed pin can complete the current pulse.
// Release the lock before waking the higher-priority sensor task.
void ARDUINO_ISR_ATTR echoEdge(void *argument) {
  int pin = int(uintptr_t(argument));
  uint32_t now = micros();
  bool done = false;

  portENTER_CRITICAL_ISR(&echoLock);
  if (pin == activePin && now - armedUs <= 16000) {
    if (gpio_get_level(gpio_num_t(pin))) {
      if (!risingSeen) {
        riseUs = now;
        risingSeen = true;
      }
    } else if (risingSeen) {
      widthUs = now - riseUs;
      activePin = -1;
      done = true;
    }
  }
  portEXIT_CRITICAL_ISR(&echoLock);

  if (done) {
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(echoDone, &wake);
    if (wake) {
      portYIELD_FROM_ISR();
    }
  }
}

// One trigger produces one reading. The semaphore lets the CPU do other
// work while sound travels to the object and back.
int measure(int sensor) {
  int pin = ECHO_PINS[sensor];
  if (digitalRead(pin) == HIGH) {
    return -4;
  }
  xSemaphoreTake(echoDone, 0);

  portENTER_CRITICAL(&echoLock);
  widthUs = 0;
  risingSeen = false;
  armedUs = micros();
  activePin = pin;
  portEXIT_CRITICAL(&echoLock);

  digitalWrite(TRIG_PINS[sensor], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PINS[sensor], LOW);
  xSemaphoreTake(echoDone, pdMS_TO_TICKS(16) + 1);

  portENTER_CRITICAL(&echoLock);
  activePin = -1;
  uint32_t duration = widthUs;
  portEXIT_CRITICAL(&echoLock);

  if (duration == 0) {
    return -1;       // No echo.
  }
  if (duration < 175) {
    return -2;      // Below usable range.
  }
  if (duration > 14577) {
    return -3;    // Above 250 cm.
  }
  // Round-trip time to one-way distance, rounded to whole centimetres.
  return (duration * 343U + 10000U) / 20000U;
}

// Schedule one transducer per slot. A full queue invalidates the old scan;
// the sequence gap tells navigation to discard incomplete evidence.
void sensorTask(void *) {
  uint32_t seq = 0;
  int sensor = LEFT;
  for (;;) {
    TickType_t start = xTaskGetTickCount();
    Reading reading{};
    reading.seq = ++seq;
    reading.at = millis();
    reading.sensor = sensor;
    reading.cm = measure(sensor);
    if (xQueueSend(readings, &reading, 0) != pdTRUE) {
      xQueueReset(readings);
      xQueueSend(readings, &reading, 0);
    }
    sensor = (sensor + 1) % N;
    vTaskDelayUntil(&start, pdMS_TO_TICKS(SENSOR_GAP_MS));
  }
}

// ================= FLOOR CALIBRATION =================
void Navigation::startFloorCalibration() {
  calibrating = true;
  floorCm = 0;
  floorCount = 0;
  floorSum = 0;
  floorLow = 251;
  floorHigh = 0;
  beam[DOWN] = Beam{};
  Serial.println("Stand still on level ground. Collecting 8 DOWN echoes.");
}

void Navigation::collectFloor(int cm) {
  floorLow = min(floorLow, cm);
  floorHigh = max(floorHigh, cm);
  if (cm < 35 || cm > 230 || floorHigh - floorLow > 10) {
    calibrating = false;
    Serial.println("Floor unstable. Check DOWN and send c again.");
    return;
  }
  floorSum += cm;
  floorCount++;
  Serial.printf("Floor calibration: %d of 8 echoes\n", floorCount);
  if (floorCount == 8) {
    floorCm = (floorSum + 4) / 8;
    calibrating = false;
    Serial.printf("Floor: %d cm.\n", floorCm);
    xQueueOverwrite(floorSaveQueue, &floorCm);
  }
}

// ================= SENSOR FILTERING =================
// Classify the latest echo immediately. Confirmation applies to reopening
// a direction; delaying a warning never keeps a blocked direction open.
void Navigation::processReading(const Reading &reading) {
  int i = reading.sensor;
  Beam &state = beam[i];
  if (reading.at - state.at >= FRESH_MS) {
    state = Beam{};
  }
  bool wasClear = state.clear;
  state.cm = reading.cm;
  state.at = reading.at;

  if (i == DOWN && calibrating) {
    collectFloor(reading.cm);
    return;
  }

  bool near = reading.cm < ENTER_CM;
  bool far = reading.cm > EXIT_CM;
  if (i == DOWN) {
    near = reading.cm < floorCm - 18;
    far = abs(reading.cm - floorCm) <= 18;
  }

  if (reading.cm < 0 || (i == DOWN && (!floorCm || (!near && !far)))) {
    state.clear = false;
    state.obstacle = false;
    state.clearCount = 0;
    return;
  }

  if (near) {
    bool possibleLimb = i == DOWN || reading.cm <= personalBand[i];
    bool centerAlsoBlocked = beam[CENTER].obstacle &&
                             reading.at - beam[CENTER].at < FRESH_MS;
    // Defer one nuisance blink only after established clearance.
    state.obstacle = !(possibleLimb && wasClear && !centerAlsoBlocked);
    state.clear = false;
    state.clearCount = 0;
  } else if (far) {
    if (state.clearCount < 2) {
      state.clearCount++;
    }
    if (state.clearCount == 2) {
      state.clear = true;
      state.obstacle = false;
    }
  } else {
    state.clearCount = 0;
  }
}

// ================= MODE 2 GUIDANCE =================
// DOWN supports all movement. Among eligible choices prefer forward,
// then a side. Keep the last turn when side ranges are within 15 cm.
uint8_t Navigation::chooseDirection(uint8_t clear) {
  if (!(clear & sensorBit(DOWN))) {
    return HOLD;
  }
  if ((clear & sensorBit(CENTER)) && (clear & sensorBit(UP))) {
    return FORWARD;
  }

  bool left = clear & sensorBit(LEFT);
  bool right = clear & sensorBit(RIGHT);
  if (left && right) {
    int difference = beam[LEFT].cm - beam[RIGHT].cm;
    if (abs(difference) <= 15 &&
        (lastDirection == TURN_LEFT || lastDirection == TURN_RIGHT)) {
      return lastDirection;
    }
    if (difference >= 0) {
      return TURN_LEFT;
    }
    return TURN_RIGHT;
  }
  if (left) {
    return TURN_LEFT;
  }
  if (right) {
    return TURN_RIGHT;
  }
  return HOLD;
}

// Each beam carries its own expiry. A heartbeat must never give a stale
// floor or direction measurement a new lease on life.
NavPacket Navigation::makePacket(uint32_t now) {
  NavPacket packet{};
  packet.magic = MAGIC;
  packet.version = VERSION;
  packet.boot = bootId;
  for (int i = 0; i < N; i++) {
    uint32_t age = now - beam[i].at;
    if (beam[i].cm < 0 || age >= FRESH_MS) {
      continue;
    }
    packet.lifeMs[i] = FRESH_MS - age;
    if (beam[i].clear) {
      packet.clearMask |= sensorBit(i);
    }
    if (beam[i].obstacle) {
      packet.obstacleMask |= sensorBit(i);
    }
  }
  lastDirection = chooseDirection(packet.clearMask);
  packet.recommendedLed = lastDirection;
  if (!floorCm) {
    packet.reason = CALIBRATE;
  }
  else if (!(packet.clearMask & sensorBit(DOWN))) {
    packet.reason = CHECK_GROUND;
  }
  else if (lastDirection == HOLD) {
    packet.reason = NO_PATH;
  }
  else {
    packet.reason = PATH_FOUND;
  }
  return packet;
}

// ================= ESP-NOW TRANSMISSION =================
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
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) == ESP_OK) {
    return true;
  }
  esp_now_deinit();
  return false;
}

// Subtract time spent waiting in the outbox before transmitting.
bool sendNavigation(const Snapshot &latest, uint32_t seq, uint32_t now) {
  NavPacket packet = latest.packet;
  packet.seq = seq;
  uint32_t age = now - latest.at;
  for (int i = 0; i < N; i++) {
    if (age >= packet.lifeMs[i]) {
      packet.lifeMs[i] = 0;
    }
    else {
      packet.lifeMs[i] -= age;
    }
  }
  return esp_now_send(PEER_MAC, (const uint8_t *)&packet, sizeof(packet)) == ESP_OK;
}

// Send on a decision change or a nominal 100 ms heartbeat. The send
// return value describes local submission; Node B monitors arrival.
void radioTask(void *) {
  Snapshot latest{};
  NavPacket previous{};
  uint32_t seq = 0;
  uint32_t lastSend = 0;
  bool ready = false;
  for (;;) {
    if (!ready) {
      ready = startRadio();
      if (!ready) {
        Serial.println("Radio unavailable. Retrying.");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
    }
    Snapshot incoming;
    if (xQueueReceive(outgoing, &incoming, pdMS_TO_TICKS(20))) {
      latest = incoming;
    }
    if (latest.packet.magic != MAGIC) {
      continue;
    }
    uint32_t now = millis();
    const NavPacket &current = latest.packet;
    bool changed = current.clearMask != previous.clearMask ||
                   current.obstacleMask != previous.obstacleMask ||
                   current.recommendedLed != previous.recommendedLed ||
                   current.reason != previous.reason;
    if (!changed && seq && now - lastSend < 100) {
      continue;
    }
    if (!sendNavigation(latest, ++seq, now)) {
      esp_now_deinit();
      ready = false;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    previous = current;
    lastSend = now;
  }
}

// ================= SERIAL DEBUGGING =================
// Read a complete diagnostic snapshot without touching live navigation state.
void printDebug() {
  DebugSnapshot snapshot{};
  if (xQueuePeek(debugQueue, &snapshot, 0) != pdTRUE) {
    Serial.println("Waiting for first scan.");
    return;
  }
  const NavPacket &packet = snapshot.packet;
  const char *choices[] = {"HOLD","FORWARD","TURN LEFT","TURN RIGHT"};
  const char *reasons[] = {"send c on level ground","check ground","no clear path","path found"};
  Serial.printf("Mode 2: %s | %s | floor %d cm\n",
                choices[packet.recommendedLed], reasons[packet.reason], snapshot.floorCm);
  Serial.printf("LEFT zone %d cm | RIGHT zone %d cm | snapshot age %lu ms\n",
                snapshot.leftZoneCm, snapshot.rightZoneCm,
                (unsigned long)(millis() - snapshot.at));
  for (int i = 0; i < N; i++) {
    const char *state = "unconfirmed";
    if (!packet.lifeMs[i]) {
      state = "unknown";
    }
    else if (packet.clearMask & sensorBit(i)) {
      state = "clear";
    }
    else if (packet.obstacleMask & sensorBit(i)) {
      state = "obstacle";
    }
    Serial.printf("%s: %d cm | %s\n", SENSOR_NAMES[i], snapshot.distances[i], state);
  }
}

void queueCommand(char key, int value) {
  Command command{key, value};
  if (xQueueSend(commandQueue, &command, 0) != pdTRUE) {
    Serial.println("Command queue busy. Repeat command.");
  }
}

void handleCommand(const char *line) {
  if (!strcmp(line,"c")) {
    queueCommand('c', 0);
  }
  else if (!strcmp(line,"s")) {
    printDebug();
  }
  else if (!strcmp(line,"v")) {
    verbose = !verbose;
    if (verbose) {
      Serial.println("Live readings enabled.");
    } else {
      Serial.println("Live readings paused.");
    }
  }
  else {
    char side, extra;
    int cm;
    if (sscanf(line,"%c %d %c",&side,&cm,&extra) == 2 &&
        (side == 'l' || side == 'r') && cm >= 0 && cm <= 40) {
      queueCommand(side, cm);
    } else {
      Serial.println("c: calibrate | s: status | v: live | l 20 / r 20: side bands");
    }
  }
}

void pollSerial() {
  static char line[28];
  static unsigned used = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      line[used] = 0;
      if (used && !overflow) {
        handleCommand(line);
      }
      used = 0;
      overflow = false;
    } else if (used < sizeof(line)-1) {
      line[used++] = c;
    } else {
      overflow = true;
    }
  }
}

// ================= SETUP AND MAIN LOOP =================
void setup() {
  Serial.begin(115200);
  readings = xQueueCreate(8, sizeof(Reading));
  outgoing = xQueueCreate(1, sizeof(Snapshot));
  commandQueue = xQueueCreate(4, sizeof(Command));
  debugQueue = xQueueCreate(1, sizeof(DebugSnapshot));
  floorSaveQueue = xQueueCreate(1, sizeof(int));
  echoDone = xSemaphoreCreateBinary();
  if (!readings || !outgoing || !echoDone || !commandQueue || !debugQueue || !floorSaveQueue) {
    ESP.restart();
  }
  for (int i = 0; i < N; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
    attachInterruptArg(ECHO_PINS[i], echoEdge, (void *)uintptr_t(ECHO_PINS[i]), CHANGE);
  }
  storageReady = storage.begin("belt-nav-v2", false);
  if (storageReady) {
    navigation.floorCm = storage.getUShort("floor", 0);
  }
  if (navigation.floorCm < 35 || navigation.floorCm > 230) {
    navigation.floorCm = 0;
  }
  WiFi.mode(WIFI_STA);
  bootId = esp_random() | 1U;
  Serial.printf("Node A %s | floor %d cm | send h for help\n", WiFi.macAddress().c_str(), navigation.floorCm);
  if (xTaskCreatePinnedToCore(
        sensorTask, "echo", 4096, nullptr, 4, nullptr, 1
      ) != pdPASS) {
    ESP.restart();
  }
  if (xTaskCreatePinnedToCore(
        navigationTask, "navigation", 4096, nullptr, 3, nullptr, 1
      ) != pdPASS) {
    ESP.restart();
  }
  if (xTaskCreatePinnedToCore(
        radioTask, "radio", 6144, nullptr, 2, nullptr, 0
      ) != pdPASS) {
    ESP.restart();
  }
}

// ================= NAVIGATION TASK =================
// This task is the only writer of Navigation after setup.
// The Serial loop receives copies for display and storage.
void navigationTask(void *) {
  uint32_t previousSequence = 0;
  for (;;) {
    Reading reading{};
    if (xQueueReceive(readings, &reading, pdMS_TO_TICKS(100)) != pdTRUE) {
      continue;
    }
    Command command{};
    while (xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
      if (command.key == 'c') {
        navigation.startFloorCalibration();
      } else {
        int sensor = LEFT;
        if (command.key == 'r') {
          sensor = RIGHT;
        }
        navigation.personalBand[sensor] = command.value;
        Serial.printf("%s personal band: %d cm\n", SENSOR_NAMES[sensor], command.value);
      }
    }
    uint32_t now = millis();
    bool expired = now - reading.at >= FRESH_MS;
    if (reading.seq != previousSequence + 1 || expired) {
      for (int i = 0; i < N; i++) {
        navigation.beam[i] = Beam{};
      }
      if (navigation.calibrating) {
        navigation.calibrating = false;
        Serial.println("Floor calibration interrupted. Send c again.");
      }
    }
    previousSequence = reading.seq;
    if (!expired) {
      navigation.processReading(reading);
    }

    Snapshot snapshot{};
    snapshot.at = millis();
    snapshot.packet = navigation.makePacket(snapshot.at);
    xQueueOverwrite(outgoing, &snapshot);

    DebugSnapshot report{};
    report.packet = snapshot.packet;
    report.at = snapshot.at;
    report.floorCm = navigation.floorCm;
    report.leftZoneCm = navigation.personalBand[LEFT];
    report.rightZoneCm = navigation.personalBand[RIGHT];
    for (int i = 0; i < N; i++) {
      report.distances[i] = navigation.beam[i].cm;
    }
    xQueueOverwrite(debugQueue, &report);
  }
}

void loop() {
  static uint32_t lastDebug = 0;
  pollSerial();
  int completedFloor;
  if (xQueueReceive(floorSaveQueue, &completedFloor, 0) == pdTRUE) {
    if (storageReady && storage.putUShort("floor", completedFloor) == sizeof(uint16_t)) {
      Serial.println("Floor reference saved for the next boot.");
    } else {
      Serial.println("Floor reference active for this boot only.");
    }
  }
  if (verbose && millis() - lastDebug >= 500) {
    lastDebug = millis();
    printDebug();
  }
  delay(10);
}
