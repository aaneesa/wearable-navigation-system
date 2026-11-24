#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ESP32-S3 N16R8 - existing wiring
constexpr int SDA_PIN = 16;
constexpr int SCL_PIN = 21;
constexpr float RAD_TO_DEG_F = 180.0f / PI;
constexpr uint32_t SAMPLE_MS = 20;
constexpr uint32_t PRINT_MS = 250;
constexpr int CAPTURE_SAMPLES = 100;
Adafruit_MPU6050 mpu;

void printAngles(float x, float y, float z) {
  float pitch = atan2f(-x, sqrtf(y*y + z*z)) * RAD_TO_DEG_F;
  float roll = atan2f(y, z) * RAD_TO_DEG_F;
  Serial.printf("Pitch: %.3f deg | Roll: %.3f deg\n", pitch, roll);
}

// Optional manual measurement only. Does not change the IMU reference.
void captureReference() {
  Serial.println("\nKeep the mounted device still in its natural standing position.");
  float sx = 0, sy = 0, sz = 0;
  float ux = 0, uy = 0, uz = 0;
  int good = 0;
  for (int i = 0; i < CAPTURE_SAMPLES; i++) {
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      float x = a.acceleration.x, y = a.acceleration.y, z = a.acceleration.z;
      float m = sqrtf(x*x + y*y + z*z);
      float w = sqrtf(g.gyro.x*g.gyro.x + g.gyro.y*g.gyro.y + g.gyro.z*g.gyro.z) * RAD_TO_DEG_F;
      // Reject obvious motion; these are diagnostic thresholds, not guarantees.
      if (fabsf(m - 9.80665f) < 0.8f && w < 5.0f) {
        sx += x; sy += y; sz += z;
        ux += x/m; uy += y/m; uz += z/m;
        good++;
      }
    }
    delay(SAMPLE_MS);
  }
  float n = sqrtf(ux*ux + uy*uy + uz*uz);
  if (good < 90 || n / good < 0.999f) {
    Serial.printf("Unstable capture (%d/100 accepted). Stand still and send c again.\n", good);
    return;
  }
  Serial.printf("\nREFERENCE (%d/100 accepted samples)\n", good);
  Serial.printf("Mean acceleration: X=%.4f Y=%.4f Z=%.4f m/s^2\n", sx/good, sy/good, sz/good);
  printAngles(ux/n, uy/n, uz/n);
  Serial.printf("const Vec3 neutralGravity = {%.6ff, %.6ff, %.6ff};\n\n", ux/n, uy/n, uz/n);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected. Check wiring and I2C address.");
    while (true) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("IMU diagnostic ready. No startup orientation calibration.");
  Serial.println("Send c in Serial Monitor to measure a fixed reference.");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') captureReference();
  }
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < PRINT_MS) return;
  lastPrint = millis();
  sensors_event_t a, g, t;
  if (!mpu.getEvent(&a, &g, &t)) {
    Serial.println("IMU read failed");
    return;
  }
  float x = a.acceleration.x, y = a.acceleration.y, z = a.acceleration.z;
  float m = sqrtf(x*x + y*y + z*z);
  Serial.printf("Accel (m/s^2): X=%+.3f Y=%+.3f Z=%+.3f | Mag=%.3f\n", x,y,z,m);
  if (isfinite(m) && m > 0.001f) {
    Serial.printf("Unit vector: X=%+.5f Y=%+.5f Z=%+.5f\n", x/m,y/m,z/m);
    printAngles(x,y,z);
  }
  Serial.printf("Gyro (deg/s): X=%+.3f Y=%+.3f Z=%+.3f\n\n",
                g.gyro.x*RAD_TO_DEG_F, g.gyro.y*RAD_TO_DEG_F, g.gyro.z*RAD_TO_DEG_F);
}
