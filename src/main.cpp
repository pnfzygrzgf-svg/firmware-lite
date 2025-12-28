#include <Arduino.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <utils/button.h>

#include <PacketSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

PacketSerial packetSerial;

#include "openbikesensor.pb.h"

// ===== UNIX time helper (real unix if set, else build time + uptime) =====
#include <time.h>
#include <sys/time.h>
#include <esp_timer.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================================
   WRAP-SAFE TIME HELPERS (uint32_t micros()/millis())
   ========================================================================== */
static inline bool time_after_u32(uint32_t a, uint32_t b) {      // a strictly after b
  return (int32_t)(a - b) > 0;
}
static inline bool time_after_eq_u32(uint32_t a, uint32_t b) {   // a after-or-equal b
  return (int32_t)(a - b) >= 0;
}
static inline uint32_t time_max_u32(uint32_t a, uint32_t b) {    // returns the later "timestamp" (wrap-safe)
  return time_after_u32(a, b) ? a : b;
}

/* ============================================================================
   BUILD-TIME -> UNIX EPOCH (UTC)
   ========================================================================== */
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

  if (tv.tv_sec > 1700000000) {
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
  }

  static const time_t build_epoch = _build_epoch_utc();
  const uint64_t boot_us = (uint64_t)esp_timer_get_time(); // µs since boot
  return (uint64_t)build_epoch * 1000000ULL + boot_us;
}

// time_source_id ist ein Identifier für die Zeitquelle (nicht links/rechts Sensor-ID!)
static inline openbikesensor_Time make_unix_time_obs(int32_t time_source_id = 1) {
  const uint64_t us = unix_time_us_now();

  openbikesensor_Time t = openbikesensor_Time_init_zero;
  t.source_id   = time_source_id;
  t.reference   = openbikesensor_Time_Reference_UNIX;
  t.seconds     = (int64_t)(us / 1000000ULL);
  t.nanoseconds = (int32_t)((us % 1000000ULL) * 1000ULL);
  return t;
}

/* ============================================================================
   BLE SETUP
   ========================================================================== */
#define OBS_BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define OBS_BLE_CHAR_TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer*         obsBleServer  = nullptr;
BLECharacteristic* obsBleTxChar  = nullptr;
bool               obsBleDeviceConnected = false;

class ObsBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    obsBleDeviceConnected = true;
    Serial.println("FW: BLE connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    obsBleDeviceConnected = false;
    Serial.println("FW: BLE disconnected -> restart advertising");
    pServer->startAdvertising();
  }
};

/* ============================================================================
   BUTTON
   ========================================================================== */
const int PUSHBUTTON_PIN = 2;
Button button(PUSHBUTTON_PIN);

/* ============================================================================
   PROTOBUF BUFFER
   ========================================================================== */
uint8_t pb_buffer[1024];
pb_ostream_t pb_ostream;

/* ============================================================================
   “Weiche” MTU-Info statt Warn-Spam
   - Nur alle X ms loggen, und als INFO
   ========================================================================== */
static uint32_t g_last_mtu_info_ms = 0;
static const uint32_t MTU_INFO_THROTTLE_MS = 2000;

static void maybe_log_len_info(size_t len) {
  if (len <= 20) return;
  const uint32_t now = millis();
  if (g_last_mtu_info_ms == 0 || time_after_eq_u32(now, g_last_mtu_info_ms + MTU_INFO_THROTTLE_MS)) {
    g_last_mtu_info_ms = now;
    Serial.printf("FW INFO: len=%u > 20 (ok if MTU negotiated; else needs framing/chunking)\n",
                  (unsigned)len);
  }
}

// gemeinsame Funktion: Protobuf-Event über BLE senden
void send_event_bytes(const uint8_t* data, size_t len) {
  Serial.printf("FW: event len=%u\n", (unsigned)len);
  maybe_log_len_info(len);

  // BLE Notification
  if (obsBleDeviceConnected && obsBleTxChar != nullptr) {
    // BLECharacteristic::setValue erwartet uint8_t* (nicht const)
    obsBleTxChar->setValue((uint8_t*)data, len);
    obsBleTxChar->notify();
  }
}

/* ============================================================================
   NANO-PB: String Callback
   ========================================================================== */
bool _write_string(pb_ostream_t* stream, const pb_field_iter_t* field, void* const* arg) {
  String& str = *((String*)(*arg));
  if (!pb_encode_tag_for_field(stream, field)) return false;
  return pb_encode_string(stream, (uint8_t*)str.c_str(), str.length());
}

void write_string(pb_callback_t& target, String& str) {
  target.arg = &str;
  target.funcs.encode = &_write_string;
}

/* ============================================================================
   EVENT SENDER
   ========================================================================== */
void send_text_message(String message, openbikesensor_TextMessage_Type type = openbikesensor_TextMessage_Type_INFO) {
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  openbikesensor_TextMessage msg = openbikesensor_TextMessage_init_zero;
  msg.type = type;
  write_string(msg.text, message);

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;
  event.content.text_message = msg;
  event.which_content = openbikesensor_Event_text_message_tag;

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  if (!pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event)) {
    Serial.println("FW ERROR: pb_encode failed for TextMessage");
    return;
  }
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

void send_distance_measurement(uint32_t source_id, float distance, uint64_t time_of_flight) {
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  openbikesensor_DistanceMeasurement distance_measurement = openbikesensor_DistanceMeasurement_init_zero;
  distance_measurement.source_id = source_id;
  distance_measurement.distance = distance;
  distance_measurement.time_of_flight = time_of_flight;

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;
  event.content.distance_measurement = distance_measurement;
  event.which_content = openbikesensor_Event_distance_measurement_tag;

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  if (!pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event)) {
    Serial.println("FW ERROR: pb_encode failed for DistanceMeasurement");
    return;
  }
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

void send_button_press() {
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  openbikesensor_UserInput user_input = openbikesensor_UserInput_init_zero;
  user_input.type = openbikesensor_UserInput_Type_OVERTAKER;
  user_input.direction = openbikesensor_UserInput_Direction_LEFT;
  user_input.timing = openbikesensor_UserInput_Timing_IMMEDIATE;

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;
  event.content.user_input = user_input;
  event.which_content = openbikesensor_Event_user_input_tag;

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  if (!pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event)) {
    Serial.println("FW ERROR: pb_encode failed for UserInput");
    return;
  }
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

void send_heartbeat() {
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  if (!pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event)) {
    Serial.println("FW ERROR: pb_encode failed for Heartbeat");
    return;
  }
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

/* ============================================================================
   SENSOR LOGIK
   ========================================================================== */
class SensorMeasurement {
public:
  uint32_t start;
  uint32_t tof;
  bool timeout = false;

  const double get_distance(const double temperature = 19.307) const {
    double speedOfSound = 20.05 * sqrt(273.16 + temperature);
    return speedOfSound * tof / 1000000.0 / 2.0;
  }
};

class Sensor {
public:
  Sensor(uint8_t source_id_, uint8_t _trigger_pin, uint8_t _echo_pin)
  : source_id(source_id_), trigger_pin(_trigger_pin), echo_pin(_echo_pin) {}

  void begin(void interrupt_echo()) {
    pinMode(echo_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(echo_pin), interrupt_echo, CHANGE);
    pinMode(trigger_pin, OUTPUT);
  }

  void echo() {
    if (digitalRead(echo_pin)) {
      start = micros();
    } else {
      end = micros();
    }
  }

  void update(bool master, Sensor& slave) {
    const uint32_t now = micros();

    uint32_t s, e;
    noInterrupts();
    s = start;
    e = end;
    interrupts();

    if (s > 0 && e > 0) {
      const uint32_t tof_us = e - s;

      measurement.start = s;
      measurement.tof = tof_us;
      measurement.timeout = tof_us > no_response_threshold;
      has_new_measurement = true;

      const uint32_t cand1 = s + interval;
      const uint32_t cand2 = now + min_delay;
      trigger_at = time_max_u32(cand1, cand2);

      triggered = 0;
      timeout_at = 0;

      noInterrupts();
      start = 0;
      end = 0;
      interrupts();
      return;
    }

    if (trigger_at > 0 && time_after_eq_u32(now, trigger_at) && master) {
      trigger_at = 0;
      triggered = now;
      timeout_at = now + timeout;

      const bool slave_ready = (slave.trigger_at > 0) && time_after_eq_u32(now, slave.trigger_at);
      if (slave_ready) {
        slave.trigger_at = 0;
        slave.triggered = now;
        slave.timeout_at = timeout_at - min_delay;
        digitalWrite(slave.trigger_pin, HIGH);
      }

      digitalWrite(trigger_pin, HIGH);
      delayMicroseconds(15);
      digitalWrite(trigger_pin, LOW);

      if (slave_ready) {
        digitalWrite(slave.trigger_pin, LOW);
      }
    }

    if (timeout_at > 0 && time_after_eq_u32(now, timeout_at)) {
      const uint32_t cand1 = triggered + interval;
      const uint32_t cand2 = now + min_delay;
      trigger_at = time_max_u32(cand1, cand2);

      triggered = 0;
      timeout_at = 0;

      noInterrupts();
      start = 0;
      end = 0;
      interrupts();

      measurement.timeout = true;
      has_new_measurement = true;
    }
  }

  uint8_t source_id;
  uint8_t trigger_pin;
  uint8_t echo_pin;

  volatile uint32_t start = 0;
  volatile uint32_t end   = 0;

  uint32_t triggered = 0;
  uint32_t trigger_at = 1;
  uint32_t timeout_at = 0;

  SensorMeasurement measurement;
  bool has_new_measurement = false;

  uint32_t interval = 40000;
  uint32_t min_delay = 5000;
  uint32_t no_response_threshold = 35000;
  uint32_t timeout = 50000;
};

Sensor sensors[] = {
  Sensor(1, 15, 4),
  Sensor(2, 25, 26),
};
uint8_t sensors_length = 2;

void IRAM_ATTR interrupt_sensor0() { sensors[0].echo(); }
void IRAM_ATTR interrupt_sensor1() { sensors[1].echo(); }

/* ============================================================================
   TIMER
   ========================================================================== */
class Timer {
public:
  Timer(uint32_t delay_) : delay(delay_) {}
  void start() { trigger_at = millis() + delay; }
  bool check() {
    if (trigger_at == 0) return false;
    const uint32_t now = millis();
    if (time_after_eq_u32(now, trigger_at)) {
      trigger_at = 0;
      return true;
    }
    return false;
  }
private:
  uint32_t trigger_at = 0;
  uint32_t delay;
};

Timer heartbeat(1000);

/* ============================================================================
   SETUP
   ========================================================================== */
void setup() {
  Serial.begin(115200);
  packetSerial.begin(115200);

  BLEDevice::init("OpenBikeSensor");
  obsBleServer = BLEDevice::createServer();
  obsBleServer->setCallbacks(new ObsBleServerCallbacks());

  BLEService* obsService = obsBleServer->createService(OBS_BLE_SERVICE_UUID);

  obsBleTxChar = obsService->createCharacteristic(
    OBS_BLE_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  obsBleTxChar->addDescriptor(new BLE2902());

  obsService->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(OBS_BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  sensors[0].begin(interrupt_sensor0);
  sensors[1].begin(interrupt_sensor1);

  heartbeat.start();

  Serial.println("FW: boot OK");
}

/* ============================================================================
   LOOP
   ========================================================================== */
void loop() {
  sensors[0].update(true,  sensors[1]);
  sensors[1].update(false, sensors[0]);

  if (heartbeat.check()) {
    send_heartbeat();
    heartbeat.start();
  }

  for (uint8_t i = 0; i < sensors_length; i++) {
    Sensor& sensor = sensors[i];

    if (sensor.has_new_measurement) {
      const SensorMeasurement& measurement = sensor.measurement;

      double distance = 99.0;
      uint32_t tof_ns = 10000;

      if (!measurement.timeout) {
        distance = measurement.get_distance();
        tof_ns = measurement.tof * 1000;
      }

      send_distance_measurement(sensor.source_id, (float)distance, (uint64_t)tof_ns);
      sensor.has_new_measurement = false;
    }
  }

  button.handle();
  if (button.gotPressed()) {
    send_button_press();
  }
}
