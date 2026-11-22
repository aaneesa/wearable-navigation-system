
# Pinout — ESP32-S3 N16R8

## Node A: Navigation

| Component | GPIO / connection |
|---|---|
| MPU6050 SDA / SCL | GPIO16 / GPIO21 (I²C) |
| Left TRIG / ECHO | GPIO4 / GPIO5 |
| Center TRIG / ECHO | GPIO6 / GPIO7 |
| Right TRIG / ECHO | GPIO14 / GPIO15 |
| Up TRIG / ECHO | GPIO10 / GPIO11 |
| Down TRIG / ECHO | GPIO12 / GPIO13 |

## Node B: Interface

| Component | GPIO |
|---|---|
| Mode 1 touch output | GPIO9 |
| Mode 2 touch output | GPIO10 |
| Forward LED1 | GPIO5 |
| Left LED2 | GPIO6 |
| Right LED3 | GPIO7 |
| Down LED4 | GPIO8 |

### Firmware Sensor Order

LEFT = 0, CENTER = 1, RIGHT = 2, UP = 3, DOWN = 4.

