#include <Arduino.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include <utils/button.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <esp_timer.h>   // esp_timer_get_time(): 64-bit uptime (µs since boot)

#include "openbikesensor.pb.h"

// ============================================================================
// Ultraschall-FW (clean minimal)
// - 2x HC-SR04/JSN-SR04T (Trigger/Echo)
// - BLE notify (OBS Lite Nordic UART UUIDs)
// - Protobuf Events: DistanceMeasurement + Heartbeat + UserInput
// - Zeit: monotonic uptime, reference = ARBITRARY (keine Fake-UNIX-Zeit)
// - Debug: ganz normal via Serial (USB-Kabel) möglich
// ============================================================================

// ------------------------- Pins / Config -------------------------
static constexpr int PUSHBUTTON_PIN = 2;

static constexpr uint8_t SENSOR0_TRIG_PIN = 15;
static constexpr uint8_t SENSOR0_ECHO_PIN = 4;

static constexpr uint8_t SENSOR1_TRIG_PIN = 25;
static constexpr uint8_t SENSOR1_ECHO_PIN = 26;

// BLE UUIDs (OBS Lite / Nordic UART style)
static constexpr const char* OBS_BLE_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char* OBS_BLE_CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

// Device name (advertised)
static constexpr const char* DEV_LOCAL_NAME = "OpenBikeSensor";

// Protobuf encode buffer
static constexpr size_t PB_BUFFER_SIZE = 1024;

// ------------------------- Wrap-safe helpers -------------------------
// micros()/millis() are uint32 and will overflow; these helpers keep comparisons safe.
static inline bool time_after_u32(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0; }
static inline bool time_after_eq_u32(uint32_t a, uint32_t b) { return (int32_t)(a - b) >= 0; }
static inline uint32_t time_max_u32(uint32_t a, uint32_t b) { return time_after_u32(a, b) ? a : b; }

// ------------------------- Time (ARBITRARY / monotonic) -------------------------
static inline openbikesensor_Time make_obs_time(int32_t time_source_id = 1) {
  const uint64_t us = (uint64_t)esp_timer_get_time();

  openbikesensor_Time t = openbikesensor_Time_init_zero;
  t.source_id   = time_source_id; // convention: 1 = internal CPU clock
  t.reference   = openbikesensor_Time_Reference_ARBITRARY;
  t.seconds     = (int64_t)(us / 1000000ULL);
  t.nanoseconds = (int32_t)((us % 1000000ULL) * 1000ULL);
  return t;
}
static inline openbikesensor_Time make_cpu_time() { return make_obs_time(1); }

// ------------------------- BLE state -------------------------
BLEServer*         g_bleServer = nullptr;
BLECharacteristic* g_bleTxChar = nullptr;
volatile bool      g_bleConnected = false;

class ObsBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    (void)s;
    g_bleConnected = true;
    Serial.println("BLE: connected");
  }
  void onDisconnect(BLEServer* s) override {
    g_bleConnected = false;
    Serial.println("BLE: disconnected -> advertising");
    s->startAdvertising();
  }
};

// ------------------------- Button -------------------------
Button button(PUSHBUTTON_PIN);

// ------------------------- Protobuf buffer -------------------------
static uint8_t      pb_buffer[PB_BUFFER_SIZE];
static pb_ostream_t pb_ostream;

// Send one encoded Event via BLE notify.
// delay(1) helps BLE stack under frequent notifications.
static inline void send_event_bytes(const uint8_t* data, size_t len) {
  if (g_bleConnected && g_bleTxChar != nullptr) {
    g_bleTxChar->setValue((uint8_t*)data, len);
    g_bleTxChar->notify();
    delay(1);
  }
}

// Encode + send an Event. Timestamp is always applied here.
static inline bool encode_and_send(openbikesensor_Event& event) {
  event.time_count = 1;
  event.time[0] = make_cpu_time();

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  const bool ok = pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
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

// ------------------------- OBS event helpers -------------------------
static inline void send_distance_measurement(uint32_t source_id, float distance_m, uint64_t tof_ns) {
  openbikesensor_DistanceMeasurement dm = openbikesensor_DistanceMeasurement_init_zero;
  dm.source_id      = source_id;
  dm.distance       = distance_m;
  dm.time_of_flight = tof_ns;

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.content.distance_measurement = dm;
  event.which_content = openbikesensor_Event_distance_measurement_tag;

  (void)encode_and_send(event);
}

static inline void send_heartbeat() {
  openbikesensor_Event event = openbikesensor_Event_init_zero;
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

// ------------------------- Wrap-safe Timer -------------------------
class Timer {
public:
  explicit Timer(uint32_t delay_ms) : delay(delay_ms) {}
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

Timer heartbeat(1000); // 1 Hz

// ------------------------- Sensor logic (HC-SR04 style) -------------------------
class SensorMeasurement {
public:
  uint32_t start = 0;
  uint32_t tof_us = 0;   // µs
  bool timeout = false;

  // Convert ToF to meters using temperature-based speed of sound
  double distance_m(double temperature_c = 19.307) const {
    const double speedOfSound = 20.05 * sqrt(273.16 + temperature_c); // m/s
    return speedOfSound * (double)tof_us / 1'000'000.0 / 2.0;
  }
};

class Sensor {
public:
  Sensor(uint8_t id, uint8_t trig, uint8_t echo)
  : source_id(id), trigger_pin(trig), echo_pin(echo) {}

  void begin(void isr()) {
    pinMode(echo_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(echo_pin), isr, CHANGE);

    pinMode(trigger_pin, OUTPUT);
    digitalWrite(trigger_pin, LOW);
  }

  // ISR handler
  void echo_edge() {
    if (digitalRead(echo_pin)) start_us = micros();
    else end_us = micros();
  }

  // Update sensor state machine.
  // Sensor0 acts as master trigger and can co-trigger the other sensor.
  void update(bool master, Sensor& slave) {
    const uint32_t now = micros();

    // Snapshot ISR timestamps atomically
    uint32_t s, e;
    noInterrupts();
    s = start_us;
    e = end_us;
    interrupts();

    // Pulse captured -> compute ToF
    if (s > 0 && e > 0) {
      const uint32_t tof = (uint32_t)(e - s);

      meas.tof_us   = tof;
      meas.timeout  = tof > no_response_threshold;
      has_new       = true;

      // schedule next trigger
      trigger_at  = time_max_u32((uint32_t)(s + interval), (uint32_t)(now + min_delay));
      triggered   = 0;
      timeout_at  = 0;

      // clear ISR timestamps
      noInterrupts();
      start_us = 0;
      end_us   = 0;
      interrupts();
      return;
    }

    // Trigger (master only)
    if (master && trigger_at > 0 && time_after_eq_u32(now, trigger_at)) {
      trigger_at = 0;
      triggered  = now;
      timeout_at = (uint32_t)(now + timeout);

      // co-trigger slave only if ready
      const bool slave_ready = (slave.trigger_at > 0) && time_after_eq_u32(now, slave.trigger_at);
      if (slave_ready) {
        slave.trigger_at = 0;
        slave.triggered  = now;
        slave.timeout_at = (uint32_t)(timeout_at - min_delay);
        digitalWrite(slave.trigger_pin, HIGH);
      }

      digitalWrite(trigger_pin, HIGH);
      delayMicroseconds(15);
      digitalWrite(trigger_pin, LOW);

      if (slave_ready) digitalWrite(slave.trigger_pin, LOW);
    }

    // Timeout (no echo)
    if (timeout_at > 0 && time_after_eq_u32(now, timeout_at)) {
      trigger_at = time_max_u32((uint32_t)(triggered + interval), (uint32_t)(now + min_delay));
      triggered  = 0;
      timeout_at = 0;

      // clear ISR timestamps
      noInterrupts();
      start_us = 0;
      end_us   = 0;
      interrupts();

      meas.timeout = true;
      meas.tof_us  = 0;
      has_new      = true;
    }
  }

  uint8_t source_id;
  uint8_t trigger_pin;
  uint8_t echo_pin;

  // ISR timestamps
  volatile uint32_t start_us = 0;
  volatile uint32_t end_us   = 0;

  // scheduling
  uint32_t trigger_at = 1;
  uint32_t triggered  = 0;
  uint32_t timeout_at = 0;

  // measurement output
  SensorMeasurement meas;
  bool has_new = false;

  // tuning
  uint32_t interval              = 40000; // 40ms
  uint32_t min_delay             = 5000;  // 5ms
  uint32_t no_response_threshold = 35000; // ~35ms
  uint32_t timeout               = 50000; // 50ms
};

// Two sensors
Sensor sensors[] = {
  Sensor(1, SENSOR0_TRIG_PIN, SENSOR0_ECHO_PIN),
  Sensor(2, SENSOR1_TRIG_PIN, SENSOR1_ECHO_PIN),
};
static constexpr uint8_t SENSORS_LEN = sizeof(sensors) / sizeof(sensors[0]);

// ISR entry points
void IRAM_ATTR isr_sensor0() { sensors[0].echo_edge(); }
void IRAM_ATTR isr_sensor1() { sensors[1].echo_edge(); }

// ============================================================================
// setup()
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(50);

  // ---- BLE init ----
  BLEDevice::init(DEV_LOCAL_NAME);

  g_bleServer = BLEDevice::createServer();
  g_bleServer->setCallbacks(new ObsBleServerCallbacks());

  BLEService* svc = g_bleServer->createService(OBS_BLE_SERVICE_UUID);

  g_bleTxChar = svc->createCharacteristic(
    OBS_BLE_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  g_bleTxChar->addDescriptor(new BLE2902());
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(OBS_BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  // ---- Sensors ----
  sensors[0].begin(isr_sensor0);
  sensors[1].begin(isr_sensor1);

  heartbeat.start();
  Serial.println("FW: boot OK");
}

// ============================================================================
// loop()
// ============================================================================
void loop() {
  // update both sensors (sensor0 is master trigger)
  sensors[0].update(true, sensors[1]);
  sensors[1].update(false, sensors[0]);

  // heartbeat 1 Hz
  if (heartbeat.check()) {
    send_heartbeat();
    heartbeat.start();
  }

  // send measurements when available
  for (uint8_t i = 0; i < SENSORS_LEN; i++) {
    Sensor& s = sensors[i];
    if (!s.has_new) continue;

    // invalid sentinel
    double distance_m = 99.0;
    uint64_t tof_ns   = 0;

    if (!s.meas.timeout) {
      distance_m = s.meas.distance_m();
      tof_ns     = (uint64_t)s.meas.tof_us * 1000ULL; // µs -> ns
    }

    send_distance_measurement(s.source_id, (float)distance_m, tof_ns);
    s.has_new = false;
  }

  // button -> UserInput event
  button.handle();
  if (button.gotPressed()) {
    send_button_press();
  }
}
