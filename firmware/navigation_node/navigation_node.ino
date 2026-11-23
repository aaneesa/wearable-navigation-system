
/*
  Wearable Navigation System - Node A
  Version: 1.3.0 (gravity-reference IMU; horizontal enclosure)

  Board: ESP32-S3 N16R8
  Sensors: 5x HC-SR04, 1x MPU6050
  Communication: ESP-NOW

  Sensor order:
  0 - LEFT
  1 - CENTER
  2 - RIGHT
  3 - UP
  4 - DOWN
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ================= CONFIGURATION =================

constexpr int N = 5;

// HC-SR04 GPIO mapping
const uint8_t TRIG_PINS[N] = {
  4, 6, 14, 10, 12
};

const uint8_t ECHO_PINS[N] = {
  5, 7, 15, 11, 13
};

// MPU6050 I2C
constexpr int SDA_PIN = 16;
constexpr int SCL_PIN = 21;

// Replace with interface ESP32 MAC for unicast.
// Broadcast is used temporarily for initial testing.
uint8_t PEER_MAC[6] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

constexpr uint8_t WIFI_CHANNEL = 1;

constexpr float MIN_CM = 3.0f;
constexpr float MAX_CM = 250.0f;

constexpr float ENTER_CM = 90.0f;
constexpr float EXIT_CM = 110.0f;

constexpr uint32_t SENSOR_GAP_MS = 55;
constexpr uint32_t TX_INTERVAL_MS = 150;
constexpr uint32_t DEBUG_INTERVAL_MS = 500;

// Actual DOWN transducer beam is 45 degrees below the HORIZONTAL enclosure.
constexpr float DOWN_MOUNT_DEG = 45.0f;
constexpr float SENSOR_HEIGHT_CM = 90.0f;
constexpr float FLOOR_MARGIN_CM = 20.0f;
constexpr float MAX_TILT_DEG = 30.0f; // Relative to wearing position
constexpr uint16_t CALIBRATION_SAMPLES = 120;
constexpr uint32_t CALIBRATION_SAMPLE_MS = 10;
// Time-based smoothing of accelerometer gravity direction (no gyro integration drift).
constexpr float GRAVITY_FILTER_TAU_SEC = 0.18f;
constexpr float GRAVITY_MIN_MS2 = 6.0f;
constexpr float GRAVITY_MAX_MS2 = 14.0f;
// Set -1 only if a controlled forward tilt shows relativePitch changing opposite to beam tilt.
constexpr float DOWN_PITCH_SIGN = 1.0f;


// ================= SENSOR IDs =================

enum SensorId {
  CENTER = 1,
  LEFT = 0,
  RIGHT = 2,
  UP = 3,
  DOWN = 4
};

const char* SENSOR_NAMES[N] = {
  "LEFT", "CENTER", "RIGHT", "UP", "DOWN"
};

// ================= PACKET STRUCTURES =================

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

  int16_t distanceCm[N];
  int16_t pitch10;
  int16_t roll10;
  uint8_t flags;
};

constexpr uint16_t MAGIC = 0x4E56;
constexpr uint8_t VERSION = 1;

constexpr uint8_t LED_FWD   = 1 << 0;
constexpr uint8_t LED_LEFT  = 1 << 1;
constexpr uint8_t LED_RIGHT = 1 << 2;
constexpr uint8_t LED_DOWN  = 1 << 3;

// ================= GLOBAL VARIABLES =================

Adafruit_MPU6050 mpu;

bool imuOK = false;

float pitchDeg = 0;
float rollDeg = 0;

uint32_t lastImuMs = 0;

volatile uint8_t selectedMode = 1;

float history[N][3] = {{0}};

uint8_t hcount[N] = {0};
uint8_t hindex[N] = {0};

float rawDistance[N] = {
  NAN, NAN, NAN, NAN, NAN
};

float filtered[N] = {
  NAN, NAN, NAN, NAN, NAN
};

uint8_t hitCount[N] = {0};
uint8_t clearCount[N] = {0};

bool obstacle[N] = {false};

uint32_t lastGoodMs[N] = {0};

uint8_t nextSensor = 0;

uint32_t lastSensorMs = 0;
uint32_t lastTxMs = 0;
uint32_t lastDebugMs = 0;

uint32_t txSeq = 0;

// The gravity vector in the IMU's fixed mounting orientation is the reference.
// Comparing complete 3-D directions avoids Euler angle wrapping and gyro drift.
struct Vec3 { float x, y, z; };
Vec3 neutralGravity = {0, 0, 1};
Vec3 filteredGravity = {0, 0, 1};
bool imuCalibrated = false;
bool gravityValid = false;
float neutralPitch = 0.0f;
float neutralRoll = 0.0f;
float relativePitch = 0.0f;
float relativeRoll = 0.0f;
float tiltAngleDeg = 0.0f;
float gravityMagnitude = 0.0f;

float clampUnit(float x) { return fmaxf(-1.0f, fminf(1.0f, x)); }
float wrapAngle(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}
float angleDiff(float current, float reference) {
  return wrapAngle(current - reference);
}
float dotVec(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 normalized(Vec3 a) {
  float n=sqrtf(dotVec(a,a));
  if(n<0.001f) return {0,0,1};
  return {a.x/n,a.y/n,a.z/n};
}

bool tiltRejected() {
  // Unknown orientation must not be classified as a clear route.
  return !imuOK || !imuCalibrated || !gravityValid ||
         tiltAngleDeg > MAX_TILT_DEG;
}

// ================= ESP-NOW RECEIVE =================

void onReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  if (len != sizeof(ModePacket)) return;

  ModePacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.magic != MAGIC) return;
  if (packet.version != VERSION) return;

  if (packet.mode != 1 && packet.mode != 2) return;

  selectedMode = packet.mode;
}

// ================= MEDIAN FILTER =================

float median3(float a, float b, float c) {

  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }

  if (b > c) {
    float t = b;
    b = c;
    c = t;
  }

  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }

  return b;
}

// ================= MPU6050 =================

void gravityAngles(Vec3 g, float &pitch, float &roll) {
  // These are IMU-frame angles for display only, NOT absolute enclosure angles.
  pitch = atan2f(-g.x, sqrtf(g.y*g.y+g.z*g.z)) * 180.0f / PI;
  roll = atan2f(g.y, g.z) * 180.0f / PI;
}

bool readGravity(Vec3 &unit, float &magnitude) {
  sensors_event_t a, gyro, temp;
  if (!mpu.getEvent(&a, &gyro, &temp)) return false;
  Vec3 v = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
  magnitude = sqrtf(dotVec(v,v));
  if (!isfinite(magnitude) || magnitude < GRAVITY_MIN_MS2 ||
      magnitude > GRAVITY_MAX_MS2) return false;
  unit = normalized(v);
  return true;
}

void calibrateIMU() {
  if (!imuOK) return;
  Serial.println("Keep the ENCLOSURE HORIZONTAL and STILL for IMU calibration...");
  Vec3 sum = {0,0,0};
  uint16_t good = 0;
  for (uint16_t i=0; i<CALIBRATION_SAMPLES; ++i) {
    Vec3 v;
    float magnitude;
    if (readGravity(v, magnitude)) {
      sum.x += v.x; sum.y += v.y; sum.z += v.z;
      good++;
    }
    delay(CALIBRATION_SAMPLE_MS);
  }
  if (good < CALIBRATION_SAMPLES * 9 / 10 ||
      sqrtf(dotVec(sum,sum)) / fmaxf(1.0f, (float)good) < 0.97f) {
    Serial.println("ERROR: IMU calibration unstable. Restart with enclosure stationary.");
    imuCalibrated = false;
    gravityValid = false;
    return;
  }
  neutralGravity = normalized(sum);
  filteredGravity = neutralGravity;
  gravityAngles(neutralGravity, neutralPitch, neutralRoll);
  pitchDeg = neutralPitch;
  rollDeg = neutralRoll;
  relativePitch = relativeRoll = tiltAngleDeg = 0.0f;
  gravityValid = true;
  imuCalibrated = true;
  lastImuMs = millis();
  Serial.print("Neutral pitch: "); Serial.println(neutralPitch,2);
  Serial.print("Neutral roll: "); Serial.println(neutralRoll,2);
  Serial.println("IMU gravity reference calibration complete");
}

void updateIMU() {
  if (!imuOK || !imuCalibrated) return;
  uint32_t now = millis();
  float dt = (now-lastImuMs)/1000.0f;
  lastImuMs = now;
  if (dt <= 0.0f || dt > 0.25f) dt = 0.02f;

  Vec3 current;
  if (!readGravity(current, gravityMagnitude)) {
    gravityValid = false;
    return;
  }
  gravityValid = true;
  // Time-based low-pass filter is independent of how often loop() is called.
  float k = dt/(GRAVITY_FILTER_TAU_SEC+dt);
  filteredGravity = normalized({
    filteredGravity.x + k*(current.x-filteredGravity.x),
    filteredGravity.y + k*(current.y-filteredGravity.y),
    filteredGravity.z + k*(current.z-filteredGravity.z)
  });
  gravityAngles(filteredGravity, pitchDeg, rollDeg);
  relativePitch = angleDiff(pitchDeg,neutralPitch);
  relativeRoll = angleDiff(rollDeg,neutralRoll);
  // True 3-D tilt relative to the calibration orientation; wrap safe.
  tiltAngleDeg = acosf(clampUnit(dotVec(filteredGravity,neutralGravity))) * 180.0f / PI;
}

// ================= ULTRASONIC READING =================

float readCm(int i) {

  // Corrected pin references
  digitalWrite(TRIG_PINS[i], LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PINS[i], HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PINS[i], LOW);

  unsigned long duration = pulseIn(
    ECHO_PINS[i],
    HIGH,
    16000UL
  );

  if (duration == 0) {
    return NAN;
  }

  float cm = duration * 0.0343f / 2.0f;

  if (cm < MIN_CM || cm > MAX_CM) {
    return NAN;
  }

  return cm;
}

// ================= FLOOR REJECTION =================

bool floorLikely(int i, float cm) {

  if (i != DOWN || !imuCalibrated || !gravityValid) {
    return false;
  }

  // Do NOT add raw IMU pitch: its built-in -25 deg mounting tilt
  // was removed by the neutral calibration. Only enclosure movement counts.
  // Pitch sign depends on the installed axis orientation.
  float downAngle =
    (DOWN_MOUNT_DEG + DOWN_PITCH_SIGN * relativePitch) *
    PI / 180.0f;

  if (downAngle < 0.20f || downAngle > 1.45f) {
    return false;
  }

  float predictedFloor =
    SENSOR_HEIGHT_CM / sinf(downAngle);

  return fabsf(cm - predictedFloor)
    < FLOOR_MARGIN_CM;
}

// ================= SENSOR FILTERING =================

void processReading(int i, float cm) {

  uint32_t now = millis();

  rawDistance[i] = cm;

  if (!isfinite(cm)) {

    if (now - lastGoodMs[i] > 1000) {
      filtered[i] = NAN;
      hitCount[i] = 0;
      clearCount[i] = 0;
      obstacle[i] = false;
      hcount[i] = 0;
      hindex[i] = 0;
    }

    return;
  }

  lastGoodMs[i] = now;

  history[i][hindex[i]] = cm;
  hindex[i] = (hindex[i] + 1) % 3;

  if (hcount[i] < 3) {
    hcount[i]++;
  }

  float med = cm;

  if (hcount[i] == 3) {
    med = median3(
      history[i][0],
      history[i][1],
      history[i][2]
    );
  }
  else if (hcount[i] == 2) {
    med = (
      history[i][0] +
      history[i][1]
    ) * 0.5f;
  }

  filtered[i] = isfinite(filtered[i])
    ? 0.55f * filtered[i] + 0.45f * med
    : med;

  bool tiltBad = tiltRejected();
  if (tiltBad) {
    hitCount[i] = 0;
    clearCount[i] = 0;
    obstacle[i] = false;
  }

  bool floor = floorLikely(
    i,
    filtered[i]
  );

  bool candidate =
    !tiltBad &&
    !floor &&
    filtered[i] < ENTER_CM;

  bool definitelyClear =
    !tiltBad &&
    (
      floor ||
      filtered[i] > EXIT_CM
    );

  uint8_t required =
    (i == LEFT || i == RIGHT) ? 3 : 2;

  if (candidate) {

    if (hitCount[i] < required) {
      hitCount[i]++;
    }

    clearCount[i] = 0;
  }
  else if (definitelyClear) {

    if (clearCount[i] < 3) {
      clearCount[i]++;
    }

    hitCount[i] = 0;
  }
  else {

    hitCount[i] = 0;
    clearCount[i] = 0;
  }

  if (hitCount[i] >= required) {
    obstacle[i] = true;
  }

  if (clearCount[i] >= 3) {
    obstacle[i] = false;
  }
}

// ================= LED MAPPING =================

uint8_t ledMaskFromObstacle(
  uint8_t obstacleMask
) {

  uint8_t result = 0;

  if (
    obstacleMask &
    ((1 << CENTER) | (1 << UP))
  ) {
    result |= LED_FWD;
  }

  if (obstacleMask & (1 << LEFT)) {
    result |= LED_LEFT;
  }

  if (obstacleMask & (1 << RIGHT)) {
    result |= LED_RIGHT;
  }

  if (obstacleMask & (1 << DOWN)) {
    result |= LED_DOWN;
  }

  return result;
}

// ================= ESP-NOW TRANSMISSION =================

void sendNavigation() {

  NavPacket packet = {};

  packet.magic = MAGIC;
  packet.version = VERSION;
  packet.mode = selectedMode;
  packet.seq = ++txSeq;

  bool tiltBad = tiltRejected();

  packet.flags =
    (imuOK ? 1 : 0) |
    (tiltBad ? 2 : 0);

  packet.pitch10 =
    (int16_t)(relativePitch * 10);

  packet.roll10 =
    (int16_t)(relativeRoll * 10);

  for (int i = 0; i < N; i++) {

    bool valid =
      isfinite(filtered[i]) &&
      millis() - lastGoodMs[i] < 800 &&
      !tiltBad;

    if (valid) {
      packet.validMask |= (1 << i);
    }

    if (valid && obstacle[i]) {
      packet.obstacleMask |= (1 << i);
    }

    packet.distanceCm[i] = valid
      ? (int16_t)roundf(filtered[i])
      : -1;
  }

  bool forwardValid =
    (packet.validMask & (1 << CENTER)) &&
    (packet.validMask & (1 << UP));

  bool leftValid =
    packet.validMask & (1 << LEFT);

  bool rightValid =
    packet.validMask & (1 << RIGHT);

  bool downValid =
    packet.validMask & (1 << DOWN);

  if (
    forwardValid &&
    !(packet.obstacleMask &
      ((1 << CENTER) | (1 << UP)))
  ) {
    packet.clearLedMask |= LED_FWD;
  }

  if (
    leftValid &&
    !(packet.obstacleMask & (1 << LEFT))
  ) {
    packet.clearLedMask |= LED_LEFT;
  }

  if (
    rightValid &&
    !(packet.obstacleMask & (1 << RIGHT))
  ) {
    packet.clearLedMask |= LED_RIGHT;
  }

  if (
    downValid &&
    !(packet.obstacleMask & (1 << DOWN))
  ) {
    packet.clearLedMask |= LED_DOWN;
  }

  // Preferred direction
  if (packet.clearLedMask & LED_DOWN) {

    if (packet.clearLedMask & LED_FWD) {
      packet.recommendedLed = 1;
    }
    else if (
      (packet.clearLedMask & LED_LEFT) &&
      (packet.clearLedMask & LED_RIGHT)
    ) {

      packet.recommendedLed =
        packet.distanceCm[LEFT] >=
        packet.distanceCm[RIGHT] ? 2 : 3;
    }
    else if (
      packet.clearLedMask & LED_LEFT
    ) {
      packet.recommendedLed = 2;
    }
    else if (
      packet.clearLedMask & LED_RIGHT
    ) {
      packet.recommendedLed = 3;
    }
  }

  if (packet.recommendedLed == 0) {
    packet.flags |= 4;
  }

  esp_err_t result = esp_now_send(
    PEER_MAC,
    (uint8_t*)&packet,
    sizeof(packet)
  );

  if (result != ESP_OK) {
    Serial.print("ESP-NOW send error: ");
    Serial.println((int)result);
  }
}

// ================= SERIAL DEBUGGING =================

void printDebug() {

  Serial.println();
  Serial.println("========== NAVIGATION DEBUG ==========");

  Serial.print("Mode: ");
  Serial.println(selectedMode);

  Serial.print("IMU: ");
  Serial.println(imuOK ? "CONNECTED" : "NOT DETECTED");

  Serial.print("Raw pitch: "); Serial.print(pitchDeg, 2);
  Serial.print(" | Raw roll: "); Serial.println(rollDeg, 2);
  Serial.print("Neutral pitch: "); Serial.print(neutralPitch, 2);
  Serial.print(" | Neutral roll: "); Serial.println(neutralRoll, 2);
  Serial.print("Relative pitch: "); Serial.print(relativePitch, 2);
  Serial.print(" | Relative roll: "); Serial.println(relativeRoll, 2);
  Serial.print("3D relative tilt: "); Serial.print(tiltAngleDeg, 2);
  Serial.print(" deg | Accelerometer magnitude: "); Serial.println(gravityMagnitude, 2);
  Serial.print("Tilt rejection: ");
  Serial.println(tiltRejected() ? "ACTIVE (READINGS INVALID)" : "NOT TRIGGERED");

  Serial.println("--------------------------------------");

  for (int i = 0; i < N; i++) {

    Serial.print(SENSOR_NAMES[i]);

    Serial.print(" | Raw: ");

    if (isfinite(rawDistance[i])) {
      Serial.print(rawDistance[i], 1);
    }
    else {
      Serial.print("INVALID");
    }

    Serial.print(" cm | Filtered: ");

    if (isfinite(filtered[i])) {
      Serial.print(filtered[i], 1);
    }
    else {
      Serial.print("INVALID");
    }

    Serial.print(" cm | Obstacle: ");

    if (tiltRejected() || !isfinite(filtered[i]) || millis()-lastGoodMs[i]>=800) {
      Serial.println("UNKNOWN (INVALID)");
    } else {
      Serial.println(obstacle[i] ? "YES" : "NO");
    }
  }

  Serial.println("======================================");
}

// ================= SETUP =================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Wearable Navigation System - Node A");
  Serial.println("Firmware Version 1.3.0");

  for (int i = 0; i < N; i++) {

    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);

    pinMode(ECHO_PINS[i], INPUT);
  }

  // Initialize MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);

  imuOK = mpu.begin();

  if (imuOK) {

    mpu.setAccelerometerRange(
      MPU6050_RANGE_4_G
    );

    mpu.setGyroRange(
      MPU6050_RANGE_500_DEG
    );

    mpu.setFilterBandwidth(
      MPU6050_BAND_21_HZ
    );

    Serial.println("MPU6050 initialized successfully");
    calibrateIMU();
  }
  else {
    Serial.println("WARNING: MPU6050 not detected");
  }

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(WIFI_CHANNEL);

  Serial.print("Navigation MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};

  memcpy(
    peer.peer_addr,
    PEER_MAC,
    6
  );

  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW peer registration failed");
  }
  else {
    Serial.println("ESP-NOW initialized");
  }

  Serial.println("Navigation system ready");
}

// ================= MAIN LOOP =================

void loop() {

  updateIMU();

  uint32_t now = millis();

  // Read one ultrasonic sensor at a time
  if (now - lastSensorMs >= SENSOR_GAP_MS) {

    lastSensorMs = now;

    float cm = readCm(nextSensor);

    processReading(
      nextSensor,
      cm
    );

    nextSensor = (nextSensor + 1) % N;
  }

  // Transmit navigation packet
  if (now - lastTxMs >= TX_INTERVAL_MS) {

    lastTxMs = now;

    sendNavigation();
  }

  // Print sensor data
  if (now - lastDebugMs >= DEBUG_INTERVAL_MS) {

    lastDebugMs = now;

    printDebug();
  }
}
