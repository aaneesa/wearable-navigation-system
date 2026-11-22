# Wearable Navigation System — v1.0.0 (Prototype)

Two ESP32-S3 N16R8 nodes communicate via ESP-NOW. The navigation node reads five HC-SR04 ultrasonic sensors (center, left, right, up, down) and one MPU6050 IMU. The interface node has two digital-output touch modules for mode selection and four directional LEDs.

> **Status:** Initial prototype; not hardware validated. Not a certified mobility aid. Do not rely on it for independent navigation or drop-off detection.

## Modes
- **Mode 1 — Obstacle:** illuminate LEDs for confirmed obstacles.
- **Mode 2 — Clear-direction:** illuminate LEDs for directions classified clear. A tentative preferred direction is also sent in the wireless packet but is not separately displayed.

| Sensor | Obstacle bit | LED |
|---|---:|---|
| Center | 0 | LED1 Forward |
| Left | 1 | LED2 Left |
| Right | 2 | LED3 Right |
| Up | 3 | LED1 Forward |
| Down | 4 | LED4 Down |

**Down LED is an awareness indicator, not a direction to walk.** Unknown readings must not be interpreted as clear.

## Repository layout
- `firmware/navigation_node/navigation_node.ino`: sensor processing, MPU6050, ESP-NOW
- `firmware/interface_node/interface_node.ino`: touch selection, LEDs, ESP-NOW
- `hardware/pinout.md`: example wiring and electrical precautions
- `docs/V1_REPORT.md`: version 1 technical report
- `experiments/TEST_PLAN.md`: 12 validation experiments
- `experiments/results/`: blank result templates; replace with measurements after testing

## Build and setup
1. Use Arduino IDE with **ESP32 Arduino core 3.x**; select the appropriate ESP32-S3 board and flash/PSRAM options for your N16R8 module.
2. Install `Adafruit MPU6050`, `Adafruit Unified Sensor`, and `Adafruit BusIO` via Library Manager.
3. Confirm your board's pin availability and wire according to `hardware/pinout.md`.
4. **Level-shift every HC-SR04 ECHO signal from 5 V to 3.3 V**; do not wire ECHO directly to the ESP32-S3.
5. Upload both sketches once and read their Wi-Fi STA MAC addresses at 115200 baud.
6. Replace `PEER_MAC` in **each** sketch with the other board's actual MAC address; reflash both. Both use ESP-NOW channel 1.
7. Verify touch polarity. Current code expects active-LOW digital touch outputs (e.g., appropriately configured TTP223 modules).
8. Calibrate belt height, down sensor angle, MPU6050 axis signs, and detection thresholds before meaningful trials.

## Current algorithm
- Sequential ultrasonic triggering (55 ms minimum between triggers), 3-sample median, EMA (alpha=0.45).
- Obstacle enter/exit thresholds: 90/110 cm; 3 hits on left/right and 2 on others; 3 clear confirmations.
- IMU complementary filter for pitch and roll; down sensor uses an approximate predicted floor distance.
- ESP-NOW mode commands sent from interface node every 500 ms; navigation packets every 150 ms.
- Interface flashes all LEDs when navigation data is stale (>750 ms) or the mode disagrees.

## Known limitations
- GPIOs are examples, not verified against a specific PCB.
- Side-arm suppression is persistence-based; it does not robustly distinguish hands from real obstacles.
- Floor model is approximate and may hide low obstacles or fail on slopes/steps.
- HC-SR04 cannot reliably detect every hazard or drop-off; no guarantee of path safety.
- MPU6050 lacks a magnetometer, so there is no stable absolute heading.
- No application-level mode acknowledgment, sequence-gap statistics, or CSV telemetry yet.
- The code has **not been compiled or tested on hardware** for this release; timings and thresholds are hypotheses.

## Next milestone
Run the tests in `experiments/TEST_PLAN.md`, commit raw logs and results, then tune thresholds and document observed performance in v1.1.

## Release
Version: `v1.0.0` · Status: prototype · Date: 2026-11-22
