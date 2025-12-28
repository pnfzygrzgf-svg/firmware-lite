#include <Arduino.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <utils/button.h>

#include <PacketSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <HardwareSerial.h>

#include "openbikesensor.pb.h"

PacketSerial packetSerial;

// ===== UNIX time helper (real unix if set, else build time + uptime) =====
#include <time.h>
#include <sys/time.h>
#include <esp_timer.h>
#include <stdlib.h>
#include <string.h>

static int _month_from_str(const char* m) {
  if (!strncmp(m, "Jan", 3)) return 0;
  if (!strncmp(m, "Feb", 3)) return 1;
  if (!strncmp(m, "Mar", 3)) return 2;
  if (!strncmp(m, "Apr", 3)) return 3;
  if (!strncmp(m, "May", 3)) return 4;
  if (!strncmp(m, "Jun", 3)) return 5;
  if (!strncmp(m, "Jul", 3)) return 6;
  if (!strncmp(m, "Aug", 3)) return 7;
  if (!strncmp(m, "Sep", 3)) return 8;
  if (!strncmp(m, "Oct", 3)) return 9;
  if (!strncmp(m, "Nov", 3)) return 10;
  if (!strncmp(m, "Dec", 3)) return 11;
  return 0;
}

static time_t _build_epoch_utc() {
  const char* date = __DATE__;  // "Mmm dd yyyy"
  const char* time_ = __TIME__; // "hh:mm:ss"

  struct tm tm_ {};
  tm_.tm_isdst = 0;

  char mon[4] = {date[0], date[1], date[2], 0};
  tm_.tm_mon  = _month_from_str(mon);
  tm_.tm_mday = atoi(date + 4);
  tm_.tm_year = atoi(date + 7) - 1900;

  tm_.tm_hour = atoi(time_ + 0);
  tm_.tm_min  = atoi(time_ + 3);
  tm_.tm_sec  = atoi(time_ + 6);

  setenv("TZ", "UTC0", 1);
  tzset();
  return mktime(&tm_);
}

static uint64_t unix_time_us_now() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  // plausible unix time (after ~2023-11)
  if (tv.tv_sec > 1700000000) {
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
  }

  static const time_t build_epoch = _build_epoch_utc();
  const uint64_t boot_us = (uint64_t)esp_timer_get_time(); // µs since boot
  return (uint64_t)build_epoch * 1000000ULL + boot_us;
}

// time_source_id is the time source identifier (NOT left/right sensor id)
static inline openbikesensor_Time make_unix_time_obs(int32_t time_source_id = 1) {
  const uint64_t us = unix_time_us_now();

  openbikesensor_Time t = openbikesensor_Time_init_zero;
  t.source_id   = time_source_id;
  t.reference   = openbikesensor_Time_Reference_UNIX;
  t.seconds     = (int64_t)(us / 1000000ULL);
  t.nanoseconds = (int32_t)((us % 1000000ULL) * 1000ULL);
  return t;
}

// ======================== BLE UUIDs (Lite) ========================
#define OBS_BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define OBS_BLE_CHAR_TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer*         obsBleServer  = nullptr;
BLECharacteristic* obsBleTxChar  = nullptr;
volatile bool      obsBleDeviceConnected = false;

class ObsBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    obsBleDeviceConnected = true;
    Serial.println("BLE: connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    obsBleDeviceConnected = false;
    Serial.println("BLE: disconnected -> advertising");
    pServer->startAdvertising();
  }
};

// ======================== Button ========================
const int PUSHBUTTON_PIN = 2;
Button button(PUSHBUTTON_PIN);

// ======================== Protobuf buffer ========================
static uint8_t      pb_buffer[1024];
static pb_ostream_t pb_ostream;

// ======================== Send helper ========================
static uint32_t g_last_send_len = 0;

void send_event_bytes(uint8_t* data, size_t len) {
  g_last_send_len = (uint32_t)len;

  // optional USB:
  // packetSerial.send(data, len);

  if (obsBleDeviceConnected && obsBleTxChar != nullptr) {
    obsBleTxChar->setValue(data, len);
    obsBleTxChar->notify();

    // give BLE stack a tiny bit of time
    delay(1);
  }
}

// ======================== Nanopb String helper ========================
bool _write_string(pb_ostream_t* stream, const pb_field_iter_t* field, void* const* arg) {
  String& str = *((String*)(*arg));
  if (!pb_encode_tag_for_field(stream, field)) return false;
  return pb_encode_string(stream, (uint8_t*)str.c_str(), str.length());
}
void write_string(pb_callback_t& target, String& str) {
  target.arg = &str;
  target.funcs.encode = &_write_string;
}

// ======================== Encode + Send with checks ========================
static inline bool encode_and_send(openbikesensor_Event& event) {
  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  bool ok = pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  if (!ok) {
    Serial.printf("PB ENCODE FAILED: %s\n", PB_GET_ERROR(&pb_ostream));
    return false;
  }
  if (pb_ostream.bytes_written == 0) {
    Serial.println("PB ENCODE WARNING: bytes_written == 0 (not sending)");
    return false;
  }
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
  return true;
}

// ======================== OBS message helpers ========================
static inline openbikesensor_Time make_cpu_time() {
  return make_unix_time_obs(1);
}

void send_text_message(String message, openbikesensor_TextMessage_Type type = openbikesensor_TextMessage_Type_INFO) {
  openbikesensor_TextMessage msg = openbikesensor_TextMessage_init_zero;
  msg.type = type;
  write_string(msg.text, message);

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = make_cpu_time();
  event.content.text_message = msg;
  event.which_content = openbikesensor_Event_text_message_tag;

  encode_and_send(event);
}

void send_distance_measurement(uint32_t source_id, float distance, uint64_t time_of_flight_ns) {
  openbikesensor_DistanceMeasurement dm = openbikesensor_DistanceMeasurement_init_zero;
  dm.source_id = source_id;
  dm.distance = distance;
  dm.time_of_flight = time_of_flight_ns;

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = make_cpu_time();
  event.content.distance_measurement = dm;
  event.which_content = openbikesensor_Event_distance_measurement_tag;

  encode_and_send(event);
}

void send_button_press() {
  openbikesensor_UserInput ui = openbikesensor_UserInput_init_zero;
  ui.type = openbikesensor_UserInput_Type_OVERTAKER;
  ui.direction = openbikesensor_UserInput_Direction_LEFT;
  ui.timing = openbikesensor_UserInput_Timing_IMMEDIATE;

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = make_cpu_time();
  event.content.user_input = ui;
  event.which_content = openbikesensor_Event_user_input_tag;

  encode_and_send(event);
}

void send_heartbeat() {
  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = make_cpu_time();
  encode_and_send(event);
}

// ======================== Timer ========================
class Timer {
public:
  Timer(uint32_t delay_) : delay(delay_) {}
  void start() { trigger_at = millis() + delay; }

  // FIX: wrap-safe check for millis() overflow
  bool check() {
    if (trigger_at != 0) {
      uint32_t now = millis();
      // wrap-safe: true if now is after-or-equal trigger_at
      if ((int32_t)(now - trigger_at) >= 0) {
        trigger_at = 0;
        return true;
      }
    }
    return false;
  }

private:
  uint32_t trigger_at = 0;
  uint32_t delay;
};

Timer heartbeat(1000);
Timer lidarSendTimer(100); // 10 Hz (stabiler als 25 Hz)

// ======================== TF-Luna (UART) ========================
static constexpr int TF_LUNA_RX_PIN = 16;
static constexpr int TF_LUNA_TX_PIN = 17;
HardwareSerial TFSerial(2);

volatile bool   lidar_has_value = false;
float           lidar_distance_m = -1.0f;
uint16_t        lidar_strength   = 0;
float           lidar_temp_c     = 0.0f;

// ===== Quality / health handling (PATCH 1) =====
static constexpr uint16_t AMP_MIN_VALID  = 100;
static constexpr uint16_t AMP_OVEREXPOSE = 65535;

// Wenn länger keine Frames kommen: Sensor "stale"
static constexpr uint32_t LIDAR_STALE_MS = 600;

// Warnungen nicht spammen
static constexpr uint32_t WARN_COOLDOWN_MS = 30000;
static uint32_t lastWarnMs  = 0;

// Zeitpunkt des letzten erfolgreich geparsten Frames
static uint32_t lastFrameMs = 0;

// Parser: 0x59 0x59 header, 9 bytes total, checksum ok
bool readTfLunaFrame(float& distance_m, uint16_t& strength, float& temp_c) {
  static uint8_t buf[9];
  static uint8_t idx = 0;

  while (TFSerial.available()) {
    uint8_t b = TFSerial.read();

    if (idx == 0) {
      if (b != 0x59) continue;
      buf[idx++] = b;
      continue;
    }
    if (idx == 1) {
      if (b != 0x59) { idx = 0; continue; }
      buf[idx++] = b;
      continue;
    }

    buf[idx++] = b;

    if (idx == 9) {
      idx = 0;

      uint16_t sum = 0;
      for (int i = 0; i < 8; i++) sum += buf[i];
      if (((uint8_t)sum) != buf[8]) {
        return false;
      }

      uint16_t dist_cm = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
      strength         = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
      uint16_t tempRaw = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
      temp_c = ((float)tempRaw) / 8.0f - 256.0f;

      // PATCH 2: Overexposure ebenfalls als ungültig behandeln
      if (strength < AMP_MIN_VALID || strength == AMP_OVEREXPOSE || dist_cm == 0 || dist_cm == 0xFFFF) {
        distance_m = -1.0f;
      } else {
        distance_m = (float)dist_cm / 100.0f;
      }

      return true;
    }
  }
  return false;
}

// ======================== Device Info strings ========================
static const char* DEV_LOCAL_NAME   = "OBS Lite LiDAR";
static const char* DEV_MANUFACTURER = "OpenBikeSensor";
static const char* DEV_FW_REV       = "OBS-Lite-LiDAR";

// ======================== Dummy right sensor ========================
static constexpr float DUMMY_RIGHT_METERS = 2.33f;

// Status 1x/s
static uint32_t lastStatusMs = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  // ================= BLE init =================
  BLEDevice::init(DEV_LOCAL_NAME);

  obsBleServer = BLEDevice::createServer();
  obsBleServer->setCallbacks(new ObsBleServerCallbacks());

  // --- Lite Service ---
  BLEService* obsService = obsBleServer->createService(OBS_BLE_SERVICE_UUID);

  obsBleTxChar = obsService->createCharacteristic(
    OBS_BLE_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  obsBleTxChar->addDescriptor(new BLE2902());

  obsService->start();

  // --- Device Information Service (0x180A) ---
  BLEService* devInfo = obsBleServer->createService(BLEUUID((uint16_t)0x180A));

  BLECharacteristic* fwRev = devInfo->createCharacteristic(
    BLEUUID((uint16_t)0x2A26),
    BLECharacteristic::PROPERTY_READ
  );
  fwRev->setValue(DEV_FW_REV);

  BLECharacteristic* manu = devInfo->createCharacteristic(
    BLEUUID((uint16_t)0x2A29),
    BLECharacteristic::PROPERTY_READ
  );
  manu->setValue(DEV_MANUFACTURER);

  devInfo->start();

  // Advertising: Lite Service UUID + scan response with name
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(OBS_BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  // ================= TF-Luna UART =================
  TFSerial.begin(115200, SERIAL_8N1, TF_LUNA_RX_PIN, TF_LUNA_TX_PIN);

  heartbeat.start();
  lidarSendTimer.start();

  // Hinweistext (wird nur gesendet wenn verbunden)
  send_text_message(String("Hello: ") + DEV_LOCAL_NAME);
}

void loop() {
  // 1) UART lesen (budgetiert)
  float d_m;
  uint16_t s;
  float t_c;

  int frames = 0;
  while (frames < 3 && readTfLunaFrame(d_m, s, t_c)) {
    frames++;
    lidar_distance_m = d_m;
    lidar_strength   = s;
    lidar_temp_c     = t_c;
    lidar_has_value  = true;

    // PATCH 3: Zeitpunkt des letzten Frames merken (für "stale" Erkennung)
    lastFrameMs      = millis();
  }

  // PATCH 3: Wenn lange kein Frame kam -> nicht alte Werte "festhalten"
  uint32_t nowMs = millis();
  if (lidar_has_value && (nowMs - lastFrameMs > LIDAR_STALE_MS)) {
    lidar_has_value = false;
    lidar_distance_m = -1.0f;
    lidar_strength = 0;

    // optional: Warnung (rate-limited)
    if (nowMs - lastWarnMs > WARN_COOLDOWN_MS) {
      send_text_message("LiDAR: no frames (UART timeout) - check wiring/power", openbikesensor_TextMessage_Type_WARNING);
      lastWarnMs = nowMs;
    }
  }

  // 2) Heartbeat
  if (heartbeat.check()) {
    send_heartbeat();
    heartbeat.start();
  }

  // 3) LiDAR senden (links) + Dummy rechts konstant
  if (lidarSendTimer.check()) {
    // PATCH 4: IMPORTER-SICHER: IMMER senden, bei ungültig mit großem Sentinelwert.
    float leftMeters = 99.0f; // "kein Messwert" -> stört min() im Import nicht

    // Nur wenn ein gültiger Messwert da ist, echten Abstand senden.
    if (lidar_has_value && lidar_distance_m > 0.0f) {
      leftMeters = lidar_distance_m;
    }

    // PATCH 1/4: ToF nicht berechnen -> 0 senden
    send_distance_measurement(1, leftMeters, 0);
    send_distance_measurement(2, DUMMY_RIGHT_METERS, 0);

    lidarSendTimer.start();
  }

  // 4) Button
  button.handle();
  if (button.gotPressed()) {
    send_button_press();
  }

  // 5) Status 1x/s
  uint32_t now = millis();
  if (now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    Serial.printf("Status: BLE=%s last_len=%lu LiDAR=%.2fm Amp=%u dummyR=%.2fm\n",
                  obsBleDeviceConnected ? "ON" : "OFF",
                  (unsigned long)g_last_send_len,
                  (double)lidar_distance_m,
                  (unsigned)lidar_strength,
                  (double)DUMMY_RIGHT_METERS);
  }
}
