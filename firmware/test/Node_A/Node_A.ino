/* Wearable Navigation System - Node A v2.0.0
   ESP32-S3 N16R8 / Arduino ESP32 Core 3.x / ESP-NOW
   Five HC-SR04 sensors. No IMU dependency.
   Prototype only: range readings do not establish a safe walking corridor. */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

constexpr int N = 5;
enum SensorId { LEFT=0, CENTER=1, RIGHT=2, UP=3, DOWN=4 };
const char* const SENSOR_NAMES[N] = {"LEFT", "CENTER", "RIGHT", "UP", "DOWN"};
const uint8_t TRIG_PINS[N] = {4, 6, 14, 10, 12};
const uint8_t ECHO_PINS[N] = {5, 7, 15, 11, 13};
// Sensor order: LEFT, CENTER, RIGHT, UP, DOWN (cm).
const float PERSONAL_CM[N] = {25, 10, 25, 10, 30};
constexpr float MIN_CM = 3.0f, MAX_CM = 250.0f;
constexpr float ENTER_CM = 90.0f, EXIT_CM = 110.0f; // existing Mode 1
constexpr float PATH_CM = 100.0f; // strictly greater than 100
constexpr uint8_t PATH_CONFIRM = 2;
constexpr uint32_t SENSOR_GAP_MS = 55, TX_INTERVAL_MS = 150;
constexpr uint32_t DEBUG_INTERVAL_MS = 500, FRESH_MS = 500;
uint8_t PEER_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // set peer MAC on both nodes for dedicated pair
constexpr uint8_t WIFI_CHANNEL = 1;
constexpr uint16_t MAGIC = 0x4E56;
constexpr uint8_t VERSION = 2;
constexpr uint8_t LED_FWD=1, LED_LEFT=2, LED_RIGHT=4, LED_DOWN=8;
constexpr uint8_t FLAG_NO_PATH = 4;
struct __attribute__((packed)) ModePacket {
  uint16_t magic; uint8_t version; uint8_t mode; uint32_t seq;
};
struct __attribute__((packed)) NavPacket {
  uint16_t magic; uint8_t version; uint8_t mode; uint32_t seq;
  uint8_t obstacleMask; uint8_t validMask; uint8_t clearLedMask;
  uint8_t recommendedLed; uint8_t personalMask;
  int16_t distanceCm[5]; uint8_t flags;
};
static_assert(sizeof(ModePacket)==8, "Mode packet layout");
static_assert(sizeof(NavPacket)==24, "Navigation packet layout");

enum ReadingState : uint8_t { UNKNOWN, PERSONAL, VALID };
struct SensorState {
  ReadingState state = UNKNOWN;
  float raw = NAN, filtered = NAN;
  float history[3] = {};
  uint8_t hcount=0, hindex=0, hits=0, clears=0, pathCount=0;
  bool obstacle=false;
  uint32_t lastSample=0;
};
// Explicit prototypes avoid Arduino auto-prototype issues with custom types.
void resetFilter(SensorState& s);
NavPacket buildNavigation(uint32_t now, uint8_t mode);
SensorState sensors[N];
portMUX_TYPE modeMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t selectedMode=1;
uint32_t txSeq=0, lastSensorMs=0, lastTxMs=0, lastDebugMs=0;
uint8_t nextSensor=0;
bool radioReady=false;

uint8_t getMode() {
  portENTER_CRITICAL(&modeMux);
  uint8_t mode=selectedMode;
  portEXIT_CRITICAL(&modeMux);
  return mode;
}
void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(ModePacket)) return;
  // In unicast mode, accept only the configured peer.
  if (PEER_MAC[0] != 0xFF && memcmp(info->src_addr, PEER_MAC, 6)!=0) return;
  ModePacket p; memcpy(&p,data,sizeof(p));
  if (p.magic!=MAGIC || p.version!=VERSION || (p.mode!=1 && p.mode!=2)) return;
  portENTER_CRITICAL(&modeMux);
  selectedMode=p.mode;
  portEXIT_CRITICAL(&modeMux);
}

float readCm(int i) {
  digitalWrite(TRIG_PINS[i], LOW); delayMicroseconds(3);
  digitalWrite(TRIG_PINS[i], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PINS[i], LOW);
  unsigned long duration=pulseIn(ECHO_PINS[i], HIGH, 16000UL);
  if (!duration) return NAN;
  return duration * 0.0343f / 2.0f;
}
float median3(float a,float b,float c) {
  if(a>b){float t=a;a=b;b=t;} if(b>c){float t=b;b=c;c=t;}
  if(a>b){float t=a;a=b;b=t;} return b;
}
void resetFilter(SensorState& s) {
  s.filtered=NAN; s.hcount=s.hindex=s.hits=s.clears=s.pathCount=0;
  s.obstacle=false;
}
void processReading(int i, float cm, uint32_t now) {
  SensorState& s=sensors[i];
  if (now-s.lastSample >= FRESH_MS) resetFilter(s);
  s.lastSample=now; s.raw=cm;
  // Never convert a missing echo or an ignored echo into a clear path.
  if (!isfinite(cm) || cm<MIN_CM || cm>MAX_CM) {
    s.state=UNKNOWN; resetFilter(s); return;
  }
  if (cm<PERSONAL_CM[i]) {
    s.state=PERSONAL; resetFilter(s); return;
  }
  s.state=VALID;
  // Independent Mode 2 evidence: two consecutive >100 cm readings.
  // Any reading <=100, ignored echo, timeout or stale sample cancels clearance.
  if (cm>PATH_CM) { if(s.pathCount<PATH_CONFIRM) ++s.pathCount; }
  else s.pathCount=0;

  // Preserve the original Mode 1 median/EMA and obstacle hysteresis.
  s.history[s.hindex]=cm; s.hindex=(s.hindex+1)%3;
  if(s.hcount<3) ++s.hcount;
  float med=cm;
  if(s.hcount==3) med=median3(s.history[0],s.history[1],s.history[2]);
  else if(s.hcount==2) med=(s.history[0]+s.history[1])*0.5f;
  s.filtered=isfinite(s.filtered)?0.55f*s.filtered+0.45f*med:med;
  uint8_t required=(i==LEFT || i==RIGHT)?3:2;
  if(s.filtered<ENTER_CM) {
    if(s.hits<required) ++s.hits;
    s.clears=0;
  } else if(s.filtered>EXIT_CM) {
    if(s.clears<3) ++s.clears;
    s.hits=0;
  } else {s.hits=0; s.clears=0;}
  if(s.hits>=required) s.obstacle=true;
  if(s.clears>=3) s.obstacle=false;
}
bool fresh(int i,uint32_t now) {
  return now-sensors[i].lastSample<FRESH_MS;
}
bool pathClear(int i,uint32_t now) {
  const SensorState& s=sensors[i];
  return fresh(i,now) && s.state==VALID && s.pathCount>=PATH_CONFIRM && s.raw>PATH_CM;
}
const char* directionName(uint8_t d) {
  switch(d){case 1:return "FORWARD";case 2:return "TURN LEFT";case 3:return "TURN RIGHT";default:return "NONE / STOP";}
}
NavPacket buildNavigation(uint32_t now,uint8_t mode) {
  NavPacket p={}; p.magic=MAGIC; p.version=VERSION; p.mode=mode;
  for(int i=0;i<N;++i) {
    const SensorState& s=sensors[i];
    bool valid=fresh(i,now) && s.state==VALID;
    if(fresh(i,now) && s.state==PERSONAL) p.personalMask|=1<<i;
    if(valid) {p.validMask|=1<<i; if(s.obstacle) p.obstacleMask|=1<<i;}
    p.distanceCm[i]=valid?(int16_t)roundf(s.raw):-1;
  }
  // DOWN is a common veto, never a direction to walk or step down.
  bool down=pathClear(DOWN,now);
  bool forward=down && pathClear(CENTER,now) && pathClear(UP,now);
  bool left=down && pathClear(LEFT,now);
  bool right=down && pathClear(RIGHT,now);
  if(down) p.clearLedMask|=LED_DOWN; // diagnostic only
  if(forward) p.clearLedMask|=LED_FWD;
  if(left) p.clearLedMask|=LED_LEFT;
  if(right) p.clearLedMask|=LED_RIGHT;
  // Prefer straight, otherwise turn toward the larger measured clearance.
  // A side cue means turn in place and rescan, NOT walk sideways.
  if(forward) p.recommendedLed=1;
  else if(left && right) p.recommendedLed=sensors[LEFT].raw>=sensors[RIGHT].raw?2:3;
  else if(left) p.recommendedLed=2;
  else if(right) p.recommendedLed=3;
  if(!p.recommendedLed) p.flags|=FLAG_NO_PATH;
  return p;
}
void sendNavigation() {
  if(!radioReady) return;
  NavPacket p=buildNavigation(millis(),getMode()); p.seq=++txSeq;
  esp_err_t result=esp_now_send(PEER_MAC,(uint8_t*)&p,sizeof(p));
  if(result!=ESP_OK) {Serial.print("ESP-NOW send error: ");Serial.println((int)result);}
}
void printDebug() {
  uint32_t now=millis(); NavPacket p=buildNavigation(now,getMode());
  Serial.println("\n========== NODE A DEBUG ==========");
  Serial.print("Mode: ");Serial.println(p.mode);
  for(int i=0;i<N;++i) {
    const SensorState& s=sensors[i];
    Serial.print(SENSOR_NAMES[i]);Serial.print(" | Raw: ");Serial.print(s.raw,1);
    Serial.print(" cm | Filtered: ");Serial.print(s.filtered,1);Serial.print(" cm | ");
    if(!fresh(i,now)||s.state==UNKNOWN) Serial.print("UNKNOWN");
    else if(s.state==PERSONAL) Serial.print("PERSONAL / IGNORED");
    else Serial.print(s.obstacle?"OBSTACLE":"NO CONFIRMED OBSTACLE");
    Serial.print(" | Path evidence: ");Serial.print(s.pathCount);Serial.print("/");Serial.print(PATH_CONFIRM);
    Serial.print(" | Beam clear: ");Serial.println(pathClear(i,now)?"YES":"NO");
  }
  Serial.print("Personal mask: 0x");Serial.println(p.personalMask,HEX);
  Serial.print("Obstacle mask: 0x");Serial.println(p.obstacleMask,HEX);
  Serial.print("Clear LED mask: 0x");Serial.println(p.clearLedMask,HEX);
  Serial.print("Recommended: ");Serial.println(directionName(p.recommendedLed));
  Serial.println("==================================");
}
void setup() {
  Serial.begin(115200);delay(1000);
  Serial.println("Wearable Navigation - Node A v2.0.0 (no IMU)");
  for(int i=0;i<N;++i){pinMode(TRIG_PINS[i],OUTPUT);digitalWrite(TRIG_PINS[i],LOW);pinMode(ECHO_PINS[i],INPUT);}
  WiFi.mode(WIFI_STA);WiFi.setChannel(WIFI_CHANNEL);
  Serial.print("Navigation MAC: ");Serial.println(WiFi.macAddress());
  if(esp_now_init()!=ESP_OK){Serial.println("ESP-NOW init failed");return;}
  if(esp_now_register_recv_cb(onReceive)!=ESP_OK){Serial.println("ESP-NOW callback failed");return;}
  esp_now_peer_info_t peer={};memcpy(peer.peer_addr,PEER_MAC,6);peer.channel=WIFI_CHANNEL;peer.encrypt=false;
  if(esp_now_add_peer(&peer)!=ESP_OK){Serial.println("ESP-NOW peer registration failed");return;}
  radioReady=true;Serial.println("Navigation ready");
}
void loop() {
  uint32_t now=millis();
  if(now-lastSensorMs>=SENSOR_GAP_MS) {
    lastSensorMs=now;float cm=readCm(nextSensor);
    processReading(nextSensor,cm,millis());nextSensor=(nextSensor+1)%N;
  }
  now=millis();
  if(now-lastTxMs>=TX_INTERVAL_MS){lastTxMs=now;sendNavigation();}
  if(now-lastDebugMs>=DEBUG_INTERVAL_MS){lastDebugMs=now;printDebug();}
}
