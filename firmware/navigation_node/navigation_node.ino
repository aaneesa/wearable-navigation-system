/*
  WEARABLE NAVIGATION SYSTEM
  NODE A - NAVIGATION BELT

  Board: ESP32-S3
  Arduino-ESP32 core: 3.x
  Serial Monitor: 115200 baud, Newline

  HARDWARE
  Five 3.3 V ultrasonic modules face left, center, right, up and down.
  Each module shares Node A's 3.3 V supply and ground.

  SOFTWARE FLOW
  1. The sensor task triggers one ultrasonic sensor.
  2. An interrupt records the rising and falling edges on ECHO.
  3. The navigation task receives the measured distance.
  4. It updates the state of that sensor and chooses a direction.
  5. The radio task sends the latest decision to Node B.

  SERIAL COMMANDS
  h       Show help.
  s       Show the latest status.
  v       Turn live readings on or off.
  c       Calibrate the floor while wearing the belt.
  l 15    Set the left personal-space band to 15 cm.
  r 20    Set the right personal-space band to 20 cm.
  l 0     Disable the left personal-space band.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>

#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_random.h>
#include <esp_task_wdt.h>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ============================================================
// 1. SENSOR NAMES, DIRECTIONS AND SETTINGS
// ============================================================

enum SensorId {
  SENSOR_LEFT,
  SENSOR_CENTER,
  SENSOR_RIGHT,
  SENSOR_UP,
  SENSOR_DOWN,
  SENSOR_COUNT
};

enum Direction {
  HOLD,
  FORWARD,
  TURN_LEFT,
  TURN_RIGHT
};

enum GuidanceReason {
  FLOOR_CALIBRATION_NEEDED,
  CHECK_GROUND,
  NO_CLEAR_DIRECTION,
  DIRECTION_AVAILABLE
};

enum CalibrationState {
  FLOOR_NOT_CALIBRATED,
  FLOOR_CALIBRATING,
  FLOOR_READY,
  FLOOR_CALIBRATION_FAILED
};

enum MeasurementError {
  NO_ECHO = -1,
  ECHO_TOO_SHORT = -2,
  ECHO_TOO_LONG = -3,
  ECHO_PIN_STUCK_HIGH = -4
};

const char *const SENSOR_NAMES[SENSOR_COUNT] = {
  "LEFT", "CENTER", "RIGHT", "UP", "DOWN"
};

const char *const DIRECTION_NAMES[] = {
  "HOLD", "FORWARD", "TURN LEFT", "TURN RIGHT"
};

const char *const REASON_NAMES[] = {
  "Floor calibration required",
  "Ground reading uncertain",
  "No confirmed clear direction",
  "Direction available"
};

// Pin order: LEFT, CENTER, RIGHT, UP, DOWN.
const uint8_t TRIGGER_PINS[SENSOR_COUNT] = {4, 6, 14, 10, 12};
const uint8_t ECHO_PINS[SENSOR_COUNT] = {5, 7, 15, 11, 13};

const uint8_t WIFI_CHANNEL = 1;

// Broadcast is convenient for the two-board bench demonstration.
// To use unicast, enter Node B's station MAC address here.
const uint8_t NODE_B_ADDRESS[6] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

const uint16_t PACKET_MAGIC = 0x4E56;
const uint8_t PACKET_VERSION = 2;

const uint32_t SENSOR_SLOT_MS = 55;
const uint32_t ECHO_TIMEOUT_US = 16000;
const uint32_t SENSOR_FRESH_MS = 450;
const uint32_t HEARTBEAT_MS = 100;

const int OBSTACLE_ENTER_CM = 90;
const int CLEAR_EXIT_CM = 110;
const int SIDE_SWITCH_MARGIN_CM = 15;

const int FLOOR_TOLERANCE_CM = 18;
const int FLOOR_SAMPLE_COUNT = 8;
const int FLOOR_MIN_CM = 35;
const int FLOOR_MAX_CM = 230;
const int FLOOR_MAX_SPREAD_CM = 10;

// ============================================================
// 2. DATA PASSED BETWEEN TASKS AND BOARDS
// ============================================================

// Keep these fields in the same order on both boards.
// remainingMs tells B how much useful life each reading has left.
struct __attribute__((packed)) NavigationPacket {
  uint32_t bootId;
  uint32_t sequence;
  uint16_t magic;
  uint8_t version;
  uint8_t direction;
  uint8_t reason;
  uint8_t warningMask;
  uint8_t clearMask;
  uint16_t remainingMs[SENSOR_COUNT];
};

static_assert(sizeof(NavigationPacket) == 25, "Packet layout changed");

struct SensorReading {
  uint32_t sequence;
  uint32_t sampledAtMs;
  uint8_t sensor;
  int16_t distanceCm;
};

struct SensorState {
  int16_t distanceCm = NO_ECHO;
  uint32_t sampledAtMs = 0;

  uint8_t consecutiveClear = 0;

  bool clear = false;
  bool warning = false;
  bool waitingForSecondNearReading = false;
};

struct NavigationSnapshot {
  NavigationPacket packet;
  uint32_t createdAtMs;
  int16_t distances[SENSOR_COUNT];

  int floorDistanceCm;
  int floorSamplesCollected;
  int calibrationState;

  int leftBandCm;
  int rightBandCm;
};

struct SensorStatistics {
  uint32_t timeouts;
  uint32_t lateSlots;
  uint32_t longestSlotUs;
};

struct RadioStatistics {
  bool ready;
  uint32_t successfulSends;
  uint32_t failedSends;
};

struct UserCommand {
  char command;
  int value;
};

// ============================================================
// 3. SHARED OBJECTS AND STATE OWNERSHIP
// ============================================================

// The sensor task writes readings. The navigation task consumes them.
QueueHandle_t readingQueue;

// Each of these queues has one slot. Its reader needs the latest value.
QueueHandle_t radioQueue;
QueueHandle_t debugQueue;
QueueHandle_t sensorStatisticsQueue;
QueueHandle_t radioStatisticsQueue;
QueueHandle_t sendResultQueue;
QueueHandle_t floorSaveQueue;

// Serial commands are applied by the navigation task.
QueueHandle_t commandQueue;

SemaphoreHandle_t echoFinishedSemaphore;
portMUX_TYPE echoLock = portMUX_INITIALIZER_UNLOCKED;

Preferences preferences;
bool preferencesAvailable = false;

uint32_t nodeBootId = 0;
bool liveDebugEnabled = false;

// After setup, only the navigation task changes the variables below.
SensorState sensors[SENSOR_COUNT];

int leftPersonalBandCm = 20;
int rightPersonalBandCm = 20;

int floorDistanceCm = 0;
int floorSamplesCollected = 0;
int lowestFloorCm = 0;
int highestFloorCm = 0;
int floorTotalCm = 0;

int calibrationState = FLOOR_NOT_CALIBRATED;
uint8_t previousDirection = HOLD;

// The echo interrupt and sensor task share only these capture fields.
volatile int armedEchoPin = -1;
volatile uint32_t captureStartedUs = 0;
volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoWidthUs = 0;
volatile bool risingEdgeReceived = false;
volatile bool completeEchoReceived = false;

// ============================================================
// 4. SMALL COMMON FUNCTIONS
// ============================================================

uint8_t sensorBit(int sensor) {
  return static_cast<uint8_t>(1U << sensor);
}

bool maskContainsSensor(uint8_t mask, int sensor) {
  return (mask & sensorBit(sensor)) != 0;
}

void restartAfterError(const char *message) {
  Serial.println(message);
  delay(1500);
  ESP.restart();
}

void prepareWatchdog() {
  if (esp_task_wdt_status(nullptr) != ESP_ERR_INVALID_STATE) {
    return;
  }

  esp_task_wdt_config_t configuration = {};
  configuration.timeout_ms = 5000;
  configuration.trigger_panic = true;

  esp_task_wdt_init(&configuration);
}

bool registerCurrentTaskWithWatchdog() {
  if (esp_task_wdt_status(nullptr) == ESP_OK) {
    return true;
  }

  if (esp_task_wdt_add(nullptr) == ESP_OK) {
    return true;
  }

  Serial.println("Watchdog unavailable for this task.");
  return false;
}

// ============================================================
// 5. ULTRASONIC ECHO CAPTURE
// ============================================================

// All five ECHO pins use this interrupt function.
// Only the pin belonging to the current measurement is accepted.
void ARDUINO_ISR_ATTR onEchoEdge(void *argument) {
  int pin = static_cast<int>(reinterpret_cast<uintptr_t>(argument));
  uint32_t nowUs = micros();
  bool notifySensorTask = false;

  portENTER_CRITICAL_ISR(&echoLock);

  if (pin == armedEchoPin) {
    uint32_t elapsedUs = nowUs - captureStartedUs;

    if (elapsedUs <= ECHO_TIMEOUT_US) {
      int level = gpio_get_level(static_cast<gpio_num_t>(pin));

      if (level == HIGH && !risingEdgeReceived) {
        echoRiseUs = nowUs;
        risingEdgeReceived = true;
      }

      if (level == LOW && risingEdgeReceived) {
        echoWidthUs = nowUs - echoRiseUs;
        completeEchoReceived = true;
        armedEchoPin = -1;
        notifySensorTask = true;
      }
    }
  }

  portEXIT_CRITICAL_ISR(&echoLock);

  if (notifySensorTask) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(
      echoFinishedSemaphore,
      &higherPriorityTaskWoken
    );

    if (higherPriorityTaskWoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }
}

void prepareEchoCapture(int pin) {
  // Remove any completion signal left by the previous measurement.
  xSemaphoreTake(echoFinishedSemaphore, 0);

  portENTER_CRITICAL(&echoLock);

  risingEdgeReceived = false;
  completeEchoReceived = false;
  echoWidthUs = 0;
  captureStartedUs = micros();
  armedEchoPin = pin;

  portEXIT_CRITICAL(&echoLock);
}

void sendTriggerPulse(int sensor) {
  digitalWrite(TRIGGER_PINS[sensor], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PINS[sensor], LOW);
}

int convertEchoToCentimeters(uint32_t durationUs) {
  if (durationUs < 175) {
    return ECHO_TOO_SHORT;
  }

  if (durationUs > 14577) {
    return ECHO_TOO_LONG;
  }

  // Distance in cm = echo time in us × 0.0343 / 2.
  // Adding 10000 rounds the integer result to the nearest cm.
  return static_cast<int>((durationUs * 343U + 10000U) / 20000U);
}

int readUltrasonicSensor(int sensor) {
  int pin = ECHO_PINS[sensor];

  if (digitalRead(pin) == HIGH) {
    return ECHO_PIN_STUCK_HIGH;
  }

  prepareEchoCapture(pin);
  sendTriggerPulse(sensor);

  // The task sleeps here. The ISR wakes it when the echo finishes.
  xSemaphoreTake(
    echoFinishedSemaphore,
    pdMS_TO_TICKS(16) + 1
  );

  portENTER_CRITICAL(&echoLock);

  armedEchoPin = -1;
  bool received = completeEchoReceived;
  uint32_t durationUs = echoWidthUs;

  portEXIT_CRITICAL(&echoLock);

  if (!received) {
    return NO_ECHO;
  }

  return convertEchoToCentimeters(durationUs);
}

void attachEchoInterrupts() {
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    uintptr_t pinNumber = ECHO_PINS[sensor];

    attachInterruptArg(
      ECHO_PINS[sensor],
      onEchoEdge,
      reinterpret_cast<void *>(pinNumber),
      CHANGE
    );
  }
}

// ============================================================
// 6. SENSOR TASK
// ============================================================

void sensorTask(void *argument) {
  attachEchoInterrupts();

  bool watchdogActive = registerCurrentTaskWithWatchdog();
  SensorStatistics statistics = {};

  uint8_t currentSensor = SENSOR_LEFT;
  uint32_t sequence = 0;
  uint32_t previousTriggerUs = 0;

  for (;;) {
    TickType_t slotStart = xTaskGetTickCount();
    uint32_t triggerUs = micros();

    if (sequence != 0) {
      uint32_t gapUs = triggerUs - previousTriggerUs;

      if (gapUs > statistics.longestSlotUs) {
        statistics.longestSlotUs = gapUs;
      }

      if (gapUs > (SENSOR_SLOT_MS + 2) * 1000U) {
        statistics.lateSlots++;
      }
    }

    previousTriggerUs = triggerUs;
    sequence++;

    SensorReading reading = {};
    reading.sequence = sequence;
    reading.sensor = currentSensor;
    reading.sampledAtMs = millis();
    reading.distanceCm = readUltrasonicSensor(currentSensor);

    if (reading.distanceCm == NO_ECHO) {
      statistics.timeouts++;
    }

    if (xQueueSend(readingQueue, &reading, 0) != pdTRUE) {
      // The consumer will see a sequence gap and clear old evidence.
      xQueueReset(readingQueue);
      xQueueSend(readingQueue, &reading, 0);
    }

    xQueueOverwrite(sensorStatisticsQueue, &statistics);

    currentSensor++;

    if (currentSensor == SENSOR_COUNT) {
      currentSensor = SENSOR_LEFT;
    }

    if (watchdogActive) {
      esp_task_wdt_reset();
    }

    // Start the next slot relative to this actual trigger.
    // This prevents a delayed scan from producing catch-up pings.
    vTaskDelayUntil(&slotStart, pdMS_TO_TICKS(SENSOR_SLOT_MS));
  }
}

// ============================================================
// 7. FLOOR CALIBRATION
// ============================================================

void startFloorCalibration() {
  calibrationState = FLOOR_CALIBRATING;

  floorDistanceCm = 0;
  floorSamplesCollected = 0;
  floorTotalCm = 0;

  lowestFloorCm = FLOOR_MAX_CM + 1;
  highestFloorCm = 0;

  sensors[SENSOR_DOWN] = SensorState{};
}

void collectFloorSample(int distanceCm) {
  if (distanceCm < FLOOR_MIN_CM || distanceCm > FLOOR_MAX_CM) {
    calibrationState = FLOOR_CALIBRATION_FAILED;
    return;
  }

  if (distanceCm < lowestFloorCm) {
    lowestFloorCm = distanceCm;
  }

  if (distanceCm > highestFloorCm) {
    highestFloorCm = distanceCm;
  }

  if (highestFloorCm - lowestFloorCm > FLOOR_MAX_SPREAD_CM) {
    calibrationState = FLOOR_CALIBRATION_FAILED;
    return;
  }

  floorTotalCm += distanceCm;
  floorSamplesCollected++;

  if (floorSamplesCollected == FLOOR_SAMPLE_COUNT) {
    floorDistanceCm =
      (floorTotalCm + FLOOR_SAMPLE_COUNT / 2) / FLOOR_SAMPLE_COUNT;

    calibrationState = FLOOR_READY;

    // The Arduino loop saves it, outside the navigation task.
    xQueueOverwrite(floorSaveQueue, &floorDistanceCm);
  }
}

bool isFloorReadingNormal(int distanceCm) {
  int difference = abs(distanceCm - floorDistanceCm);
  return difference <= FLOOR_TOLERANCE_CM;
}

// ============================================================
// 8. OBSTACLE AND CLEARANCE CLASSIFICATION
// ============================================================

bool isInsidePersonalBand(int sensor, int distanceCm) {
  if (sensor == SENSOR_LEFT && leftPersonalBandCm > 0) {
    return distanceCm <= leftPersonalBandCm;
  }

  if (sensor == SENSOR_RIGHT && rightPersonalBandCm > 0) {
    return distanceCm <= rightPersonalBandCm;
  }

  return false;
}

bool centerAlsoSeesObstacle(uint32_t sampleTimeMs) {
  const SensorState &center = sensors[SENSOR_CENTER];

  if (!center.warning) {
    return false;
  }

  return sampleTimeMs - center.sampledAtMs < SENSOR_FRESH_MS;
}

void recordCloseReading(int sensor) {
  SensorState &state = sensors[sensor];

  bool wasClear = state.clear;
  bool alreadyNear = state.warning || state.waitingForSecondNearReading;

  bool canDeferWarning = isInsidePersonalBand(sensor, state.distanceCm);

  if (sensor == SENSOR_DOWN) {
    canDeferWarning = true;
  }

  // A suspected hand or leg still blocks movement guidance immediately.
  state.clear = false;
  state.consecutiveClear = 0;

  bool waitOneReading = canDeferWarning &&
                        wasClear &&
                        !alreadyNear &&
                        !centerAlsoSeesObstacle(state.sampledAtMs);

  if (waitOneReading) {
    state.warning = false;
    state.waitingForSecondNearReading = true;
  } else {
    state.warning = true;
    state.waitingForSecondNearReading = false;
  }
}

void recordClearReading(int sensor) {
  SensorState &state = sensors[sensor];

  state.waitingForSecondNearReading = false;

  if (state.consecutiveClear < 2) {
    state.consecutiveClear++;
  }

  if (state.consecutiveClear >= 2) {
    state.clear = true;
    state.warning = false;
  }
}

void updateSensorState(const SensorReading &reading) {
  int sensor = reading.sensor;
  SensorState &state = sensors[sensor];

  if (reading.sampledAtMs - state.sampledAtMs >= SENSOR_FRESH_MS) {
    state = SensorState{};
  }

  state.distanceCm = reading.distanceCm;
  state.sampledAtMs = reading.sampledAtMs;

  if (sensor == SENSOR_DOWN && calibrationState == FLOOR_CALIBRATING) {
    collectFloorSample(reading.distanceCm);
    state.clear = false;
    state.warning = false;
    state.consecutiveClear = 0;
    state.waitingForSecondNearReading = false;
    return;
  }

  if (reading.distanceCm < 0 ||
      (sensor == SENSOR_DOWN && calibrationState != FLOOR_READY)) {
    state.clear = false;
    state.warning = false;
    state.consecutiveClear = 0;
    state.waitingForSecondNearReading = false;
    return;
  }

  if (sensor == SENSOR_DOWN) {
    if (isFloorReadingNormal(reading.distanceCm)) {
      recordClearReading(sensor);
    } else if (reading.distanceCm < floorDistanceCm - FLOOR_TOLERANCE_CM) {
      recordCloseReading(sensor);
    } else {
      // A longer-than-expected return leaves the ground uncertain.
      state.clear = false;
      state.warning = false;
      state.consecutiveClear = 0;
      state.waitingForSecondNearReading = false;
    }

    return;
  }

  if (reading.distanceCm < OBSTACLE_ENTER_CM) {
    recordCloseReading(sensor);
    return;
  }

  if (reading.distanceCm > CLEAR_EXIT_CM) {
    recordClearReading(sensor);
    return;
  }

  // Between 90 and 110 cm, retain the established state.
  // A reading in this band cannot establish a new clear direction.
  state.consecutiveClear = 0;
  state.waitingForSecondNearReading = false;
}

// ============================================================
// 9. MODE 2 - CHOOSE ONE DIRECTION
// ============================================================

uint8_t chooseSideDirection(bool leftClear, bool rightClear) {
  if (leftClear && rightClear) {
    int differenceCm =
      sensors[SENSOR_LEFT].distanceCm - sensors[SENSOR_RIGHT].distanceCm;

    if (abs(differenceCm) <= SIDE_SWITCH_MARGIN_CM) {
      if (previousDirection == TURN_LEFT ||
          previousDirection == TURN_RIGHT) {
        return previousDirection;
      }
    }

    if (differenceCm >= 0) {
      return TURN_LEFT;
    }

    return TURN_RIGHT;
  }

  if (leftClear) {
    return TURN_LEFT;
  }

  if (rightClear) {
    return TURN_RIGHT;
  }

  return HOLD;
}

uint8_t chooseDirection(uint8_t clearMask) {
  if (!maskContainsSensor(clearMask, SENSOR_DOWN)) {
    return HOLD;
  }

  bool centerClear = maskContainsSensor(clearMask, SENSOR_CENTER);
  bool upperClear = maskContainsSensor(clearMask, SENSOR_UP);

  if (centerClear && upperClear) {
    return FORWARD;
  }

  bool leftClear = maskContainsSensor(clearMask, SENSOR_LEFT);
  bool rightClear = maskContainsSensor(clearMask, SENSOR_RIGHT);

  return chooseSideDirection(leftClear, rightClear);
}

uint8_t explainDirection(uint8_t direction, uint8_t clearMask) {
  if (calibrationState != FLOOR_READY) {
    return FLOOR_CALIBRATION_NEEDED;
  }

  if (!maskContainsSensor(clearMask, SENSOR_DOWN)) {
    return CHECK_GROUND;
  }

  if (direction == HOLD) {
    return NO_CLEAR_DIRECTION;
  }

  return DIRECTION_AVAILABLE;
}

void buildNavigationSnapshot(NavigationSnapshot &snapshot) {
  uint32_t nowMs = millis();

  snapshot = NavigationSnapshot{};
  snapshot.createdAtMs = nowMs;
  snapshot.floorDistanceCm = floorDistanceCm;
  snapshot.floorSamplesCollected = floorSamplesCollected;
  snapshot.calibrationState = calibrationState;
  snapshot.leftBandCm = leftPersonalBandCm;
  snapshot.rightBandCm = rightPersonalBandCm;

  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    const SensorState &state = sensors[sensor];
    snapshot.distances[sensor] = state.distanceCm;

    uint32_t ageMs = nowMs - state.sampledAtMs;

    if (state.distanceCm < 0 || ageMs >= SENSOR_FRESH_MS) {
      continue;
    }

    snapshot.packet.remainingMs[sensor] = SENSOR_FRESH_MS - ageMs;

    if (state.clear) {
      snapshot.packet.clearMask |= sensorBit(sensor);
    }

    if (state.warning) {
      snapshot.packet.warningMask |= sensorBit(sensor);
    }
  }

  snapshot.packet.direction = chooseDirection(snapshot.packet.clearMask);

  snapshot.packet.reason = explainDirection(
    snapshot.packet.direction,
    snapshot.packet.clearMask
  );

  previousDirection = snapshot.packet.direction;
}

void processUserCommands() {
  UserCommand command;

  while (xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
    if (command.command == 'c') {
      startFloorCalibration();
    } else if (command.command == 'l') {
      leftPersonalBandCm = command.value;
    } else if (command.command == 'r') {
      rightPersonalBandCm = command.value;
    }
  }
}

void navigationTask(void *argument) {
  bool watchdogActive = registerCurrentTaskWithWatchdog();
  uint32_t previousSequence = 0;

  for (;;) {
    SensorReading reading;

    if (xQueueReceive(readingQueue, &reading, pdMS_TO_TICKS(100)) != pdTRUE) {
      continue;
    }

    processUserCommands();

    bool missedReading = reading.sequence != previousSequence + 1;
    bool readingExpired = millis() - reading.sampledAtMs >= SENSOR_FRESH_MS;

    if (missedReading || readingExpired) {
      for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
        sensors[sensor] = SensorState{};
      }

      if (calibrationState == FLOOR_CALIBRATING) {
        calibrationState = FLOOR_CALIBRATION_FAILED;
      }
    }

    previousSequence = reading.sequence;

    if (!readingExpired) {
      updateSensorState(reading);
    }

    NavigationSnapshot snapshot;
    buildNavigationSnapshot(snapshot);

    xQueueOverwrite(radioQueue, &snapshot);
    xQueueOverwrite(debugQueue, &snapshot);

    if (watchdogActive) {
      esp_task_wdt_reset();
    }
  }
}

// ============================================================
// 10. ESP-NOW TRANSMISSION
// ============================================================

// Arduino-ESP32 3.x releases use two different types for the first
// send-callback argument. This small adapter accepts either type.
// We only need the second argument: the transmission result.
struct RadioCallback {
  template <typename AddressOrSendInfo>
  static void onSendFinished(
    const AddressOrSendInfo *unused,
    esp_now_send_status_t result
  ) {
    xQueueOverwrite(sendResultQueue, &result);
  }
};

bool initializeRadio() {
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

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, NODE_B_ADDRESS, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_register_send_cb(RadioCallback::onSendFinished) != ESP_OK) {
    esp_now_deinit();
    return false;
  }

  if (esp_now_add_peer(&peer) != ESP_OK) {
    esp_now_deinit();
    return false;
  }

  return true;
}

bool guidanceChanged(
  const NavigationPacket &current,
  const NavigationPacket &previous
) {
  if (current.direction != previous.direction) {
    return true;
  }

  if (current.reason != previous.reason) {
    return true;
  }

  if (current.warningMask != previous.warningMask) {
    return true;
  }

  return current.clearMask != previous.clearMask;
}

void subtractWaitingTime(
  NavigationPacket &packet,
  uint32_t snapshotAgeMs
) {
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    uint16_t remaining = packet.remainingMs[sensor];

    if (snapshotAgeMs >= remaining) {
      packet.remainingMs[sensor] = 0;
    } else {
      packet.remainingMs[sensor] = remaining - snapshotAgeMs;
    }
  }
}

void radioTask(void *argument) {
  bool watchdogActive = registerCurrentTaskWithWatchdog();

  NavigationSnapshot latest = {};
  NavigationPacket previous = {};
  RadioStatistics statistics = {};

  uint32_t sequence = 0;
  uint32_t lastSendMs = 0;
  unsigned int consecutiveFailures = 0;

  for (;;) {
    if (watchdogActive) {
      esp_task_wdt_reset();
    }

    if (!statistics.ready) {
      xQueueReset(sendResultQueue);
      statistics.ready = initializeRadio();
      xQueueOverwrite(radioStatisticsQueue, &statistics);

      if (!statistics.ready) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
    }

    NavigationSnapshot incoming;

    if (xQueueReceive(radioQueue, &incoming, pdMS_TO_TICKS(20)) == pdTRUE) {
      latest = incoming;
    }

    uint32_t nowMs = millis();
    NavigationPacket packet = latest.packet;

    bool heartbeatDue = nowMs - lastSendMs >= HEARTBEAT_MS;

    if (sequence != 0 && !heartbeatDue && !guidanceChanged(packet, previous)) {
      continue;
    }

    subtractWaitingTime(packet, nowMs - latest.createdAtMs);

    sequence++;
    packet.magic = PACKET_MAGIC;
    packet.version = PACKET_VERSION;
    packet.bootId = nodeBootId;
    packet.sequence = sequence;

    previous = packet;
    lastSendMs = nowMs;

    esp_err_t sendResult = esp_now_send(
      NODE_B_ADDRESS,
      reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet)
    );

    bool submitted = sendResult == ESP_OK;
    bool callbackReceived = false;
    esp_now_send_status_t completion = ESP_NOW_SEND_FAIL;

    if (submitted) {
      callbackReceived = xQueueReceive(
        sendResultQueue,
        &completion,
        pdMS_TO_TICKS(80)
      ) == pdTRUE;
    }

    if (callbackReceived && completion == ESP_NOW_SEND_SUCCESS) {
      statistics.successfulSends++;
      consecutiveFailures = 0;
    } else {
      statistics.failedSends++;
      consecutiveFailures++;

      bool callbackTimedOut = submitted && !callbackReceived;

      if (callbackTimedOut || consecutiveFailures >= 4) {
        esp_now_deinit();
        statistics.ready = false;
        consecutiveFailures = 0;
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }

    xQueueOverwrite(radioStatisticsQueue, &statistics);
  }
}

// ============================================================
// 11. SERIAL MONITOR
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("========== NODE A COMMANDS ==========");
  Serial.println("h      Show help");
  Serial.println("s      Show current status");
  Serial.println("v      Toggle live sensor readings");
  Serial.println("c      Calibrate floor while wearing belt");
  Serial.println("l 20   Set LEFT personal band to 20 cm");
  Serial.println("r 20   Set RIGHT personal band to 20 cm");
  Serial.println("Bands accept 0 to 40 cm. Zero disables the band.");
  Serial.println("Floor calibration is saved. Side bands reset at boot.");
  Serial.println("=====================================");
}

const char *describeReading(const NavigationSnapshot &snapshot, int sensor) {
  int distance = snapshot.distances[sensor];

  if (distance == NO_ECHO) {
    return "NO ECHO";
  }

  if (distance == ECHO_TOO_SHORT) {
    return "BELOW RANGE";
  }

  if (distance == ECHO_TOO_LONG) {
    return "ABOVE RANGE";
  }

  if (distance == ECHO_PIN_STUCK_HIGH) {
    return "ECHO PIN STUCK HIGH";
  }

  if (snapshot.packet.remainingMs[sensor] == 0) {
    return "EXPIRED";
  }

  if (maskContainsSensor(snapshot.packet.warningMask, sensor)) {
    return "OBSTACLE";
  }

  if (maskContainsSensor(snapshot.packet.clearMask, sensor)) {
    return "CLEAR";
  }

  return "UNCONFIRMED";
}

void printStatus() {
  NavigationSnapshot snapshot;

  if (xQueuePeek(debugQueue, &snapshot, 0) != pdTRUE) {
    Serial.println("Waiting for first measurement.");
    return;
  }

  Serial.println();
  Serial.println("========== NODE A STATUS ==========");

  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Direction: ");
  Serial.println(DIRECTION_NAMES[snapshot.packet.direction]);

  Serial.print("Reason: ");
  Serial.println(REASON_NAMES[snapshot.packet.reason]);

  Serial.printf("Floor reference: %d cm\n", snapshot.floorDistanceCm);
  Serial.printf("Personal bands: LEFT %d cm | RIGHT %d cm\n",
                snapshot.leftBandCm, snapshot.rightBandCm);

  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    Serial.print(SENSOR_NAMES[sensor]);
    Serial.print(" | ");

    if (snapshot.distances[sensor] >= 0) {
      Serial.print(snapshot.distances[sensor]);
      Serial.print(" cm | ");
    }

    Serial.println(describeReading(snapshot, sensor));
  }

  SensorStatistics scan = {};
  RadioStatistics radio = {};

  xQueuePeek(sensorStatisticsQueue, &scan, 0);
  xQueuePeek(radioStatisticsQueue, &radio, 0);

  Serial.printf("Echo timeouts: %lu | Late slots: %lu | Longest slot: %lu us\n",
                static_cast<unsigned long>(scan.timeouts),
                static_cast<unsigned long>(scan.lateSlots),
                static_cast<unsigned long>(scan.longestSlotUs));

  if (radio.ready) {
    Serial.println("Radio: READY");
  } else {
    Serial.println("Radio: RETRYING");
  }

  Serial.printf("Send completions: %lu | Failures: %lu\n",
                static_cast<unsigned long>(radio.successfulSends),
                static_cast<unsigned long>(radio.failedSends));

  Serial.println("===================================");
}

void handleSerialCommand(const char *line) {
  if (strcmp(line, "h") == 0 || strcmp(line, "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(line, "s") == 0) {
    printStatus();
    return;
  }

  if (strcmp(line, "v") == 0) {
    liveDebugEnabled = !liveDebugEnabled;

    if (liveDebugEnabled) {
      Serial.println("Live readings ON.");
    } else {
      Serial.println("Live readings OFF.");
    }

    return;
  }

  UserCommand command = {};

  if (strcmp(line, "c") == 0) {
    command.command = 'c';
  } else {
    char extraCharacter;

    int fields = sscanf(
      line,
      "%c %d %c",
      &command.command,
      &command.value,
      &extraCharacter
    );

    bool validSide = command.command == 'l' || command.command == 'r';
    bool validValue = command.value >= 0 && command.value <= 40;

    if (fields != 2 || !validSide || !validValue) {
      Serial.println("Unknown command or value. Send h for help.");
      return;
    }
  }

  if (xQueueSend(commandQueue, &command, 0) != pdTRUE) {
    Serial.println("Command queue busy. Please repeat the command.");
    return;
  }

  if (command.command == 'c') {
    Serial.println("Stand still on level ground. Keep DOWN unobstructed.");
    Serial.println("Collecting eight floor readings...");
  } else {
    Serial.printf("Requested personal band: %c = %d cm\n",
                  command.command, command.value);
  }
}

void readSerialInput() {
  static char line[24];
  static size_t length = 0;
  static bool overflow = false;

  while (Serial.available() > 0) {
    char character = Serial.read();

    if (character == '\r' || character == '\n') {
      if (overflow) {
        Serial.println("Command too long. Send h for help.");
      } else if (length > 0) {
        line[length] = '\0';
        handleSerialCommand(line);
      }

      length = 0;
      overflow = false;
    } else if (length < sizeof(line) - 1) {
      line[length] = character;
      length++;
    } else {
      overflow = true;
    }
  }
}

void reportCalibrationProgress() {
  static int previousState = -1;
  static int previousCount = -1;

  NavigationSnapshot snapshot;

  if (xQueuePeek(debugQueue, &snapshot, 0) != pdTRUE) {
    return;
  }

  if (snapshot.calibrationState == FLOOR_CALIBRATING) {
    if (previousState != FLOOR_CALIBRATING ||
        previousCount != snapshot.floorSamplesCollected) {
      Serial.printf("Floor calibration: %d/%d\n",
                    snapshot.floorSamplesCollected, FLOOR_SAMPLE_COUNT);
    }
  } else if (snapshot.calibrationState != previousState) {
    if (snapshot.calibrationState == FLOOR_NOT_CALIBRATED) {
      Serial.println("Floor reference missing. Fit the belt and send c.");
    }

    if (snapshot.calibrationState == FLOOR_CALIBRATION_FAILED) {
      Serial.println("Calibration failed. Check DOWN, stand still and send c.");
    }
  }

  previousState = snapshot.calibrationState;
  previousCount = snapshot.floorSamplesCollected;
}

void saveCompletedCalibration() {
  int newFloorCm;

  if (xQueueReceive(floorSaveQueue, &newFloorCm, 0) != pdTRUE) {
    return;
  }

  bool saved = false;

  if (preferencesAvailable) {
    size_t bytesWritten = preferences.putUShort("floor", newFloorCm);
    saved = bytesWritten == sizeof(uint16_t);
  }

  Serial.printf("Floor calibration complete: %d cm. ", newFloorCm);

  if (saved) {
    Serial.println("Saved for next boot.");
  } else {
    Serial.println("Active for this boot only.");
  }
}

// ============================================================
// 12. SETUP AND ARDUINO LOOP
// ============================================================

void configureSensorPins() {
  for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    pinMode(TRIGGER_PINS[sensor], OUTPUT);
    digitalWrite(TRIGGER_PINS[sensor], LOW);
    pinMode(ECHO_PINS[sensor], INPUT);
  }
}

void loadFloorCalibration() {
  preferencesAvailable = preferences.begin("belt-nav-v2", false);

  if (!preferencesAvailable) {
    return;
  }

  int storedFloor = preferences.getUShort("floor", 0);

  if (storedFloor >= FLOOR_MIN_CM && storedFloor <= FLOOR_MAX_CM) {
    floorDistanceCm = storedFloor;
    calibrationState = FLOOR_READY;
  }
}

void createCommunicationObjects() {
  readingQueue = xQueueCreate(8, sizeof(SensorReading));
  radioQueue = xQueueCreate(1, sizeof(NavigationSnapshot));
  debugQueue = xQueueCreate(1, sizeof(NavigationSnapshot));
  sensorStatisticsQueue = xQueueCreate(1, sizeof(SensorStatistics));
  radioStatisticsQueue = xQueueCreate(1, sizeof(RadioStatistics));
  sendResultQueue = xQueueCreate(1, sizeof(esp_now_send_status_t));
  floorSaveQueue = xQueueCreate(1, sizeof(int));
  commandQueue = xQueueCreate(4, sizeof(UserCommand));

  echoFinishedSemaphore = xSemaphoreCreateBinary();

  if (!readingQueue || !radioQueue || !debugQueue ||
      !sensorStatisticsQueue || !radioStatisticsQueue ||
      !sendResultQueue || !floorSaveQueue || !commandQueue ||
      !echoFinishedSemaphore) {
    restartAfterError("Node A could not allocate queues or semaphore.");
  }
}

void startNodeATasks() {
  BaseType_t result;

  result = xTaskCreatePinnedToCore(
    sensorTask, "Ultrasonic", 4096, nullptr, 4, nullptr, 1
  );

  if (result != pdPASS) {
    restartAfterError("Could not start ultrasonic task.");
  }

  result = xTaskCreatePinnedToCore(
    navigationTask, "Navigation", 4096, nullptr, 3, nullptr, 1
  );

  if (result != pdPASS) {
    restartAfterError("Could not start navigation task.");
  }

  result = xTaskCreatePinnedToCore(
    radioTask, "Radio", 6144, nullptr, 2, nullptr, 0
  );

  if (result != pdPASS) {
    restartAfterError("Could not start radio task.");
  }
}

void setup() {
  Serial.begin(115200);

  configureSensorPins();
  loadFloorCalibration();

  WiFi.mode(WIFI_STA);
  nodeBootId = esp_random() | 1U;

  Serial.println();
  Serial.println("WEARABLE NAVIGATION - NODE A");
  Serial.print("Station MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Stored floor reference: %d cm\n", floorDistanceCm);

  printHelp();
  createCommunicationObjects();
  prepareWatchdog();
  startNodeATasks();
}

void loop() {
  static uint32_t lastDebugMs = 0;

  readSerialInput();
  saveCompletedCalibration();
  reportCalibrationProgress();

  if (liveDebugEnabled && millis() - lastDebugMs >= 500) {
    lastDebugMs = millis();
    printStatus();
  }

  delay(10);
}