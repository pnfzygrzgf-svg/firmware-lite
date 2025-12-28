#include <Arduino.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <utils/button.h>

#include <PacketSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <esp_timer.h>   // 64-bit uptime (µs since boot)
#include <math.h>

#include "openbikesensor.pb.h"

PacketSerial packetSerial;

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
   TIME (UPTIME ONLY) correct semantic: ARBITRARY
   - No fake UNIX, no build time.
   - Monotonic since boot, 64-bit.
   ========================================================================== */
static inline openbikesensor_Time make_obs_time(int32_t time_source_id = 1) {
  const uint64_t us = (uint64_t)esp_timer_get_time();

  openbikesensor_Time t = openbikesensor_Time_init_zero;
  t.source_id   = time_source_id; // 1 = internal CPU clock (your convention)
  t.reference   = openbikesensor_Time_Reference_ARBITRARY;
  t.seconds     = (int64_t)(us / 1000000ULL);
  t.nanoseconds = (int32_t)((us % 1000000ULL) * 1000ULL);
  return t;
}

static inline openbikesensor_Time make_cpu_time() {
  return make_obs_time(1);
}

/* ============================================================================
   BLE SETUP
   ========================================================================== */
#define OBS_BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define OBS_BLE_CHAR_TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer*         obsBleServer  = nullptr;
BLECharacteristic* obsBleTxChar  = nullptr;
volatile bool      obsBleDeviceConnected = false;

class ObsBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    (void)pServer;
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
static uint8_t      pb_buffer[1024];
static pb_ostream_t pb_ostream;

/* ============================================================================
   SEND RAW EVENT BYTES (BLE notify)
   ========================================================================== */
static inline void send_event_bytes(const uint8_t* data, size_t len) {
  if (obsBleDeviceConnected && obsBleTxChar != nullptr) {
    obsBleTxChar->setValue((uint8_t*)data, len);
    obsBleTxChar->notify();
    delay(1); // (1) give BLE stack a tiny bit of time
  }
}

/* ============================================================================
   NANO-PB: String Callback
   ========================================================================== */
static bool _write_string(pb_ostream_t* stream, const pb_field_iter_t* field, void* const* arg) {
  String& str = *((String*)(*arg));
  if (!pb_encode_tag_for_field(stream, field)) return false;
  return pb_encode_string(stream, (uint8_t*)str.c_str(), str.length());
}

static inline void write_string(pb_callback_t& target, String& str) {
  target.arg = &str;
  target.funcs.encode = &_write_string;
}

/* ============================================================================
   ENCODE + SEND (Timestamp zentral gesetzt)
   ========================================================================== */
static inline bool encode_and_send(openbikesensor_Event& event) {
  event.time_count = 1;
  event.time[0] = make_cpu_time();

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  const bool ok = pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  if (!ok) {
    Serial.printf("FW ERROR: pb_encode failed: %s\n", PB_GET_ERROR(&pb_ostream));
    return false;
  }
  if (pb_ostream.bytes_written == 0) {
    Serial.println("FW WARN: pb_encode bytes_written == 0 (not sending)");
    return false;
  }

  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
  return true;
}

/* ============================================================================
   OBS EVENT HELPERS
   ========================================================================== */
static inline void send_text_message(String message,
                                    openbikesensor_TextMessage_Type type = openbikesensor_TextMessage_Type_INFO) {
  openbikesensor_TextMessage msg = openbikesensor_TextMessage_init_zero;
  msg.type = type;
  write_string(msg.text, message);

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.content.text_message = msg;
  event.which_content = openbikesensor_Event_text_message_tag;

  (void)encode_and_send(event);
}

static inline void send_distance_measurement(uint32_t source_id, float distance, uint64_t time_of_flight_ns) {
  openbikesensor_DistanceMeasurement dm = openbikesensor_DistanceMeasurement_init_zero;
  dm.source_id      = source_id;
  dm.distance       = distance;
  dm.time_of_flight = time_of_flight_ns;

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.content.distance_measurement = dm;
  event.which_content = openbikesensor_Event_distance_measurement_tag;

  (void)encode_and_send(event);
}

static inline void send_button_press() {
  openbikesensor_UserInput ui = openbikesensor_UserInput_init_zero;
  ui.type      = openbikesensor_UserInput_Type_OVERTAKER;
  ui.direction = openbikesensor_UserInput_Direction_LEFT;
  ui.timing    = openbikesensor_UserInput_Timing_IMMEDIATE;

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.content.user_input = ui;
  event.which_content = openbikesensor_Event_user_input_tag;

  (void)encode_and_send(event);
}

static inline void send_heartbeat() {
  openbikesensor_Event event = openbikesensor_Event_init_zero;
  (void)encode_and_send(event);
}

/* ============================================================================
   SENSOR LOGIK
   ========================================================================== */
class SensorMeasurement {
public:
  uint32_t start = 0;
  uint32_t tof = 0; // microseconds
  bool timeout = false;

  double get_distance(const double temperature = 19.307) const {
    // temperature in degree celsius, returns meters
    double speedOfSound = 20.05 * sqrt(273.16 + temperature);
    // factor 2.0 because the sound travels the distance twice
    return speedOfSound * (double)tof / 1000000.0 / 2.0;
  }
};

class Sensor {
public:
  Sensor(uint8_t source_id_, uint8_t trigger_pin_, uint8_t echo_pin_)
  : source_id(source_id_), trigger_pin(trigger_pin_), echo_pin(echo_pin_) {}

  void begin(void interrupt_echo()) {
    pinMode(echo_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(echo_pin), interrupt_echo, CHANGE);
    pinMode(trigger_pin, OUTPUT);
    digitalWrite(trigger_pin, LOW);
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

    // Got pulse
    if (s > 0 && e > 0) {
      const uint32_t tof_us = (uint32_t)(e - s);

      measurement.start = s;
      measurement.tof = tof_us;
      measurement.timeout = tof_us > no_response_threshold;
      has_new_measurement = true;

      const uint32_t cand1 = (uint32_t)(s + interval);
      const uint32_t cand2 = (uint32_t)(now + min_delay);
      trigger_at = time_max_u32(cand1, cand2);

      triggered = 0;
      timeout_at = 0;

      noInterrupts();
      start = 0;
      end = 0;
      interrupts();
      return;
    }

    // Trigger (master triggers, optionally also slave)
    if (trigger_at > 0 && time_after_eq_u32(now, trigger_at) && master) {
      trigger_at = 0;
      triggered = now;
      timeout_at = (uint32_t)(now + timeout);

      const bool slave_ready = (slave.trigger_at > 0) && time_after_eq_u32(now, slave.trigger_at);
      if (slave_ready) {
        slave.trigger_at = 0;
        slave.triggered = now;
        slave.timeout_at = (uint32_t)(timeout_at - min_delay);
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
    if (timeout_at > 0 && time_after_eq_u32(now, timeout_at)) {
      const uint32_t cand1 = (uint32_t)(triggered + interval);
      const uint32_t cand2 = (uint32_t)(now + min_delay);
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

  uint32_t triggered  = 0;
  uint32_t trigger_at = 1;
  uint32_t timeout_at = 0;

  SensorMeasurement measurement;
  bool has_new_measurement = false;

  uint32_t interval              = 40000; // target 40ms interval
  uint32_t min_delay             = 5000;  // min delay 5ms between echo and next trigger
  uint32_t no_response_threshold = 35000; // ~35ms
  uint32_t timeout               = 50000; // timeout µs
};

Sensor sensors[] = {
  Sensor(1, 15, 4),
  Sensor(2, 25, 26),
};
uint8_t sensors_length = 2;

void IRAM_ATTR interrupt_sensor0() { sensors[0].echo(); }
void IRAM_ATTR interrupt_sensor1() { sensors[1].echo(); }

/* ============================================================================
   TIMER (millis wrap-safe)
   ========================================================================== */
class Timer {
public:
  explicit Timer(uint32_t delay_) : delay(delay_) {}
  void start() { trigger_at = (uint32_t)millis() + delay; }
  bool check() {
    if (trigger_at == 0) return false;
    const uint32_t now = (uint32_t)millis();
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
  packetSerial.begin(115200); // optional, not used for sending in this sketch

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

      double distance_m = 99.0;     // sentinel for invalid
      uint32_t tof_ns   = 0;        // invalid => 0

      if (!measurement.timeout) {
        distance_m = measurement.get_distance();
        tof_ns = measurement.tof * 1000; // µs -> ns
      }

      send_distance_measurement(sensor.source_id, (float)distance_m, (uint64_t)tof_ns);
      sensor.has_new_measurement = false;
    }
  }

  button.handle();
  if (button.gotPressed()) {
    send_button_press();
  }

  // packetSerial.update(); // only needed if you receive framed serial data
}
