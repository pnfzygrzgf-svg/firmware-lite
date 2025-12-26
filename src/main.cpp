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

// --- BLE Setup ---

// UUIDs für Service & TX-Characteristic (Notify -> iPhone)
#define OBS_BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define OBS_BLE_CHAR_TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer*         obsBleServer  = nullptr;
BLECharacteristic* obsBleTxChar  = nullptr;
bool               obsBleDeviceConnected = false;

class ObsBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    obsBleDeviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    obsBleDeviceConnected = false;
    // Wieder Werbung starten, damit ein iPhone neu verbinden kann
    pServer->startAdvertising();
  }
};

// Button config
const int PUSHBUTTON_PIN = 2;
int numButtonReleased = 0;
Button button(PUSHBUTTON_PIN);

uint8_t pb_buffer[1024];
pb_ostream_t pb_ostream;

// gemeinsame Funktion: Protobuf-Event über USB-Serial UND BLE senden
void send_event_bytes(uint8_t* data, size_t len) {
  // DEBUG: Länge ausgeben (das ist der Test!)
  Serial.printf("ESP SEND len=%u\n", (unsigned)len);

  // 1) wie bisher über PacketSerial (USB)
  // packetSerial.send(data, len);

  // 2) zusätzlich als BLE-Notification
  if (obsBleDeviceConnected && obsBleTxChar != nullptr) {
    obsBleTxChar->setValue(data, len);
    obsBleTxChar->notify();
  }
}

bool _write_string(pb_ostream_t* stream, const pb_field_iter_t* field, void* const* arg) {
  String& str = *((String*)(*arg));

  if (!pb_encode_tag_for_field(stream, field)) {
    return false;
  }

  return pb_encode_string(stream, (uint8_t*)str.c_str(), str.length());
}

void write_string(pb_callback_t& target, String& str) {
  target.arg = &str;
  target.funcs.encode = &_write_string;
}

void send_text_message(String message, openbikesensor_TextMessage_Type type = openbikesensor_TextMessage_Type_INFO) {
  // create the time (UNIX)
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  // create the text message
  openbikesensor_TextMessage msg = openbikesensor_TextMessage_init_zero;
  msg.type = type;
  write_string(msg.text, message);

  // create the event
  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;
  event.content.text_message = msg;
  event.which_content = openbikesensor_Event_text_message_tag;

  // write out
  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

void send_distance_measurement(uint32_t source_id, float distance, uint64_t time_of_flight) {
  // create the time (UNIX)
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  // create the distance measurement
  openbikesensor_DistanceMeasurement distance_measurement = openbikesensor_DistanceMeasurement_init_zero;
  distance_measurement.source_id = source_id;
  distance_measurement.distance = distance;
  distance_measurement.time_of_flight = time_of_flight;

  // create the event
  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;
  event.content.distance_measurement = distance_measurement;
  event.which_content = openbikesensor_Event_distance_measurement_tag;

  // write out
  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

void send_button_press() {
  // create the time (UNIX)
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  openbikesensor_UserInput user_input = openbikesensor_UserInput_init_zero;
  user_input.type = openbikesensor_UserInput_Type_OVERTAKER;
  user_input.direction = openbikesensor_UserInput_Direction_LEFT;
  user_input.timing = openbikesensor_UserInput_Timing_IMMEDIATE;

  // create the event
  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;
  event.content.user_input = user_input;
  event.which_content = openbikesensor_Event_user_input_tag;

  // write out
  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

void send_heartbeat() {
  // create the time (UNIX)
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  // create the event
  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;

  // write out
  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

class SensorMeasurement {
public:
  uint32_t start;
  uint32_t tof; // microseconds
  bool timeout = false;

  const double get_distance(const double temperature = 19.307) const {
    // temperature in degree celsius, returns meters
    double speedOfSound = 20.05 * sqrt(273.16 + temperature);
    return speedOfSound * tof / 1000000.0 / 2.0;
  }
};

class Sensor {
public:
  Sensor(uint8_t source_id_, uint8_t _trigger_pin, uint8_t _echo_pin) :
    source_id(source_id_),
    trigger_pin(_trigger_pin),
    echo_pin(_echo_pin) {
  }

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

  void update(bool master, Sensor slave) {
    uint32_t now = micros();

    // Read response
    if (start > 0 && end > 0) {
      uint32_t tof = end - start;
      measurement.start = start;
      measurement.tof = end - start;
      measurement.timeout = tof > no_response_threshold;
      has_new_measurement = true;

      trigger_at = max(start + interval, now + min_delay);
      triggered = 0;
      start = 0;
      end = 0;
      timeout_at = 0;
      return;
    }

    // Trigger
    if (trigger_at > 0 && now > trigger_at && master) {
      trigger_at = 0;
      triggered = now;
      timeout_at = now + timeout;

      bool slave_ready = (slave.trigger_at > 0) && (now > slave.trigger_at);
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

    // Timeout
    if (timeout_at > 0 && now > timeout_at) {
      trigger_at = max(triggered + interval, now + min_delay);
      triggered = 0;
      start = 0;
      end = 0;
      timeout_at = 0;

      measurement.timeout = true;
      has_new_measurement = true;
    }
  }

  bool measuring;

  uint8_t source_id;
  uint8_t trigger_pin;
  uint8_t echo_pin;
  uint32_t start = 0;
  uint32_t triggered = 0;
  uint32_t end = 0;
  uint32_t trigger_at = 1;
  uint32_t timeout_at = 0;

  SensorMeasurement measurement;
  bool has_new_measurement;

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

class Timer {
public:
  Timer(uint32_t delay_) : delay(delay_) {}

  void start() { trigger_at = millis() + delay; }

  bool check() {
    if (trigger_at <= millis()) {
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

void setup() {
  Serial.begin(115200);       // <-- wichtig für Serial Monitor
  packetSerial.begin(115200);

  // --- BLE initialisieren ---
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
  // --- Ende BLE-Init ---

  sensors[0].begin(interrupt_sensor0);
  sensors[1].begin(interrupt_sensor1);

  heartbeat.start();

  // OPTIONAL: einmalig große Message senden, um sicher >20 Bytes zu testen
  send_text_message(String(200, 'A'));
}

void loop() {
  for (uint8_t i = 0; i < sensors_length; i++) {
    sensors[i].update(i == 0, sensors[1]);
  }

  if (heartbeat.check()) {
    send_heartbeat();
    heartbeat.start();
  }

  for (uint8_t i = 0; i < sensors_length; i++) {
    Sensor& sensor = sensors[i];

    if (sensor.has_new_measurement) {
      SensorMeasurement const& measurement = sensor.measurement;

      double distance = 99.0;
      uint32_t tof = 10000;

      if (!measurement.timeout) {
        distance = measurement.get_distance();
        tof = measurement.tof * 1000;
      }

      send_distance_measurement(sensor.source_id, distance, tof);
      sensor.has_new_measurement = false;
    }
  }

  button.handle();

  if (button.gotPressed()) {
    send_button_press();
  }

  // packetSerial.update();
}
