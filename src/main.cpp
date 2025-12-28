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
   ----------------------------------------------------------------------------
   Arduino/ESP32 liefert micros() / millis() als uint32_t.
   Diese Zähler laufen über:
     - micros():  ~71.6 Minuten
     - millis():  ~49.7 Tage

   Direkte Vergleiche wie "now > t" sind beim Wrap falsch.
   Der Standard-Trick: (int32_t)(now - t) >= 0 ist wrap-sicher,
   solange die Zeitdifferenzen < 2^31 Ticks bleiben
     - micros: < ~35.8 Minuten
     - millis: < ~24.8 Tage
   Für typische Timeouts/Intervalle (ms…sek) ist das völlig safe.
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
   ----------------------------------------------------------------------------
   Falls der ESP32 keine echte RTC/NTP-Zeit hat, gibt gettimeofday() sehr kleine
   Werte (tv_sec ~ 0) zurück. Dann bauen wir uns eine "plausible" UNIX-Zeit:
     build time (__DATE__/__TIME__) + uptime (esp_timer_get_time()).
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

  // mktime() nutzt lokale TZ -> wir setzen explizit UTC
  setenv("TZ", "UTC0", 1);
  tzset();
  return mktime(&tm_);
}

static uint64_t unix_time_us_now() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  // plausible unix time (after ~2023-11). Wenn ja: verwende echte Zeit.
  if (tv.tv_sec > 1700000000) {
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
  }

  // Sonst: Build-Zeit + Uptime
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
   ----------------------------------------------------------------------------
   Wir senden Protobuf-Events per BLE Notification an das iPhone.
   Achtung: ohne MTU/Chunking sind typischerweise nur 20 Bytes Payload möglich!
   ========================================================================== */

// UUIDs für Service & TX-Characteristic (Notify -> iPhone) (NUS-ähnlich)
#define OBS_BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define OBS_BLE_CHAR_TX_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer*         obsBleServer  = nullptr;
BLECharacteristic* obsBleTxChar  = nullptr;
bool               obsBleDeviceConnected = false;

// Callback: merkt sich, ob ein Gerät verbunden ist
class ObsBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    obsBleDeviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    obsBleDeviceConnected = false;
    // Werbung neu starten, damit ein iPhone erneut verbinden kann
    pServer->startAdvertising();
  }
};

/* ============================================================================
   BUTTON
   ----------------------------------------------------------------------------
   Hinweis: GPIO2 ist beim ESP32 ein Strapping-Pin (Bootmodus).
   Wenn der Button beim Boot den Pegel beeinflusst, kann der ESP32 evtl. nicht
   sauber starten. Besser ggf. anderen Pin wählen.
   ========================================================================== */
const int PUSHBUTTON_PIN = 2;
int numButtonReleased = 0;
Button button(PUSHBUTTON_PIN);

/* ============================================================================
   PROTOBUF BUFFER
   ----------------------------------------------------------------------------
   Wir encoden openbikesensor_Event in pb_buffer und senden ihn dann.
   ========================================================================== */
uint8_t pb_buffer[1024];
pb_ostream_t pb_ostream;

// gemeinsame Funktion: Protobuf-Event über USB-Serial UND BLE senden
void send_event_bytes(const uint8_t* data, size_t len) {
  // DEBUG: Länge ausgeben (hilft beim Testen / BLE-MTU-Problem)
  Serial.printf("ESP SEND len=%u\n", (unsigned)len);

  // 1) optional per USB/PacketSerial (derzeit auskommentiert)
  // packetSerial.send(data, len);

  // 2) BLE Notification
  if (obsBleDeviceConnected && obsBleTxChar != nullptr) {
    obsBleTxChar->setValue(data, len);
    obsBleTxChar->notify();
  }
}

/* ============================================================================
   NANO-PB: String Callback
   ----------------------------------------------------------------------------
   openbikesensor.pb.h verwendet für Strings üblicherweise pb_callback_t.
   Damit NanoPB den Arduino String encoden kann, geben wir einen encode-callback
   an.
   ========================================================================== */
bool _write_string(pb_ostream_t* stream, const pb_field_iter_t* field, void* const* arg) {
  String& str = *((String*)(*arg));

  // Tag für dieses Feld schreiben
  if (!pb_encode_tag_for_field(stream, field)) {
    return false;
  }

  // Bytes (UTF-8) encoden
  return pb_encode_string(stream, (uint8_t*)str.c_str(), str.length());
}

void write_string(pb_callback_t& target, String& str) {
  target.arg = &str;
  target.funcs.encode = &_write_string;
}

/* ============================================================================
   EVENT SENDER (Text / Distance / Button / Heartbeat)
   ========================================================================== */
void send_text_message(String message, openbikesensor_TextMessage_Type type = openbikesensor_TextMessage_Type_INFO) {
  // Zeitstempel erstellen (UNIX)
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  // TextMessage erstellen
  openbikesensor_TextMessage msg = openbikesensor_TextMessage_init_zero;
  msg.type = type;
  write_string(msg.text, message);

  // Event erstellen
  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;
  event.content.text_message = msg;
  event.which_content = openbikesensor_Event_text_message_tag;

  // Encoden + senden
  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

void send_distance_measurement(uint32_t source_id, float distance, uint64_t time_of_flight) {
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  openbikesensor_DistanceMeasurement distance_measurement = openbikesensor_DistanceMeasurement_init_zero;
  distance_measurement.source_id = source_id;   // Sensor-ID (z.B. links/rechts)
  distance_measurement.distance = distance;     // Meter
  distance_measurement.time_of_flight = time_of_flight; // Nano-Sek? (bei dir: tof*1000)

  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;
  event.content.distance_measurement = distance_measurement;
  event.which_content = openbikesensor_Event_distance_measurement_tag;

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
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
  pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

void send_heartbeat() {
  openbikesensor_Time cpu_time = make_unix_time_obs(1);

  // Heartbeat = Event ohne content (nur time)
  openbikesensor_Event event = openbikesensor_Event_init_zero;
  event.time_count = 1;
  event.time[0] = cpu_time;

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
}

/* ============================================================================
   SENSOR LOGIK
   ----------------------------------------------------------------------------
   Wir messen Time-of-Flight (ToF) mit einem Ultraschall-Sensor:
     - Trigger-Pin: kurzer Puls triggert Messung
     - Echo-Pin: HIGH während Schall unterwegs ist (ToF = HIGH-Dauer)

   Wichtig:
     - echo() wird aus dem Interrupt aufgerufen (CHANGE am Echo-Pin)
     - start/end werden im ISR gesetzt und im loop() gelesen
       => volatile + atomar lesen/setzen (Race-Condition Fix)
   ========================================================================== */
class SensorMeasurement {
public:
  uint32_t start;    // Zeitpunkt (micros) als Echo HIGH begann
  uint32_t tof;      // time-of-flight in µs
  bool timeout = false;

  // Distanz aus ToF berechnen (Temperatur optional)
  const double get_distance(const double temperature = 19.307) const {
    // speedOfSound in m/s (Approx)
    double speedOfSound = 20.05 * sqrt(273.16 + temperature);
    // Strecke = v * t / 2 (hin+zurück)
    return speedOfSound * tof / 1000000.0 / 2.0;
  }
};

class Sensor {
public:
  Sensor(uint8_t source_id_, uint8_t _trigger_pin, uint8_t _echo_pin)
  : source_id(source_id_), trigger_pin(_trigger_pin), echo_pin(_echo_pin) {}

  void begin(void interrupt_echo()) {
    // Echo wird per Interrupt gemessen
    pinMode(echo_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(echo_pin), interrupt_echo, CHANGE);

    // Trigger-Pin treibt den Sensor
    pinMode(trigger_pin, OUTPUT);
  }

  // ISR: bei Echo HIGH -> start setzen; bei Echo LOW -> end setzen
  void echo() {
    if (digitalRead(echo_pin)) {
      start = micros();
    } else {
      end = micros();
    }
  }

  // update() wird im loop aufgerufen:
  //  - master = true: darf triggern
  //  - slave = Referenz auf anderen Sensor (FIX: nicht by-value!)
  void update(bool master, Sensor& slave) {
    const uint32_t now = micros();

    // --- Atomar start/end auslesen (ISR-sicher) ---
    // Dadurch verhindern wir, dass ISR während des Lesens dazwischenfunkt.
    uint32_t s, e;
    noInterrupts();
    s = start;
    e = end;
    interrupts();

    // 1) Antwort fertig? (start und end gesetzt)
    if (s > 0 && e > 0) {
      const uint32_t tof_us = e - s;

      // Messung speichern
      measurement.start = s;
      measurement.tof = tof_us;
      measurement.timeout = tof_us > no_response_threshold;
      has_new_measurement = true;

      // Nächsten Trigger terminieren:
      // mindestens interval nach start ODER min_delay nach now (wrap-sicheres max)
      const uint32_t cand1 = s + interval;
      const uint32_t cand2 = now + min_delay;
      trigger_at = time_max_u32(cand1, cand2);

      // Status zurücksetzen
      triggered = 0;
      timeout_at = 0;

      // start/end atomar zurücksetzen (wieder ISR-sicher)
      noInterrupts();
      start = 0;
      end = 0;
      interrupts();
      return;
    }

    // 2) Trigger auslösen (nur Master) (wrap-sicherer Vergleich)
    if (trigger_at > 0 && time_after_eq_u32(now, trigger_at) && master) {
      trigger_at = 0;
      triggered = now;
      timeout_at = now + timeout;

      // Optional: Slave gleichzeitig triggern, wenn auch bereit
      const bool slave_ready = (slave.trigger_at > 0) && time_after_eq_u32(now, slave.trigger_at);
      if (slave_ready) {
        slave.trigger_at = 0;
        slave.triggered = now;
        // Slave bekommt leicht früheren Timeout (min_delay)
        slave.timeout_at = timeout_at - min_delay;
        digitalWrite(slave.trigger_pin, HIGH);
      }

      // Trigger-Puls erzeugen
      digitalWrite(trigger_pin, HIGH);
      delayMicroseconds(15);
      digitalWrite(trigger_pin, LOW);

      if (slave_ready) {
        digitalWrite(slave.trigger_pin, LOW);
      }
    }

    // 3) Timeout: kein Echo bekommen (wrap-sicherer Vergleich)
    if (timeout_at > 0 && time_after_eq_u32(now, timeout_at)) {
      // Nächster Trigger
      const uint32_t cand1 = triggered + interval;
      const uint32_t cand2 = now + min_delay;
      trigger_at = time_max_u32(cand1, cand2);

      // Status reset
      triggered = 0;
      timeout_at = 0;

      // start/end ebenfalls resetten (Sicherheit)
      noInterrupts();
      start = 0;
      end = 0;
      interrupts();

      // Timeout-Messung markieren
      measurement.timeout = true;
      has_new_measurement = true;
    }
  }

  // ------------------ Konfiguration / State ------------------
  bool measuring = false;

  uint8_t source_id;
  uint8_t trigger_pin;
  uint8_t echo_pin;

  // Shared ISR<->loop state:
  // volatile => Compiler optimiert Zugriff nicht weg
  volatile uint32_t start = 0;
  volatile uint32_t end   = 0;

  // Scheduler-Variablen (micros-basiert)
  uint32_t triggered = 0;
  uint32_t trigger_at = 1;  // wann darf als nächstes getriggert werden
  uint32_t timeout_at = 0;  // wann läuft die Messung spätestens aus

  SensorMeasurement measurement;
  bool has_new_measurement = false;

  // Timing Parameter (µs)
  uint32_t interval = 40000;
  uint32_t min_delay = 5000;
  uint32_t no_response_threshold = 35000;
  uint32_t timeout = 50000;
};

// Zwei Sensoren: source_id 1 und 2, mit Trigger/Echo Pins
Sensor sensors[] = {
  Sensor(1, 15, 4),
  Sensor(2, 25, 26),
};
uint8_t sensors_length = 2;

// ISR wrapper müssen IRAM_ATTR haben (ESP32), damit sie im IRAM liegen
void IRAM_ATTR interrupt_sensor0() { sensors[0].echo(); }
void IRAM_ATTR interrupt_sensor1() { sensors[1].echo(); }

/* ============================================================================
   TIMER (Heartbeat)
   ----------------------------------------------------------------------------
   Heartbeat alle 1000ms. Auch hier wrap-sicher, weil millis() überläuft.
   ========================================================================== */
class Timer {
public:
  Timer(uint32_t delay_) : delay(delay_) {}

  // startet Timer: "jetzt + delay"
  void start() { trigger_at = millis() + delay; }

  // liefert true, wenn fällig; danach reset auf 0
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

  // --- BLE initialisieren ---
  BLEDevice::init("OpenBikeSensor");

  obsBleServer = BLEDevice::createServer();
  obsBleServer->setCallbacks(new ObsBleServerCallbacks());

  BLEService* obsService = obsBleServer->createService(OBS_BLE_SERVICE_UUID);

  // TX Characteristic: nur Notify (iPhone empfängt)
  obsBleTxChar = obsService->createCharacteristic(
    OBS_BLE_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  // CCCD Descriptor damit iPhone Notifications aktivieren kann
  obsBleTxChar->addDescriptor(new BLE2902());

  obsService->start();

  // Advertising konfigurieren
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(OBS_BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  // --- Ende BLE-Init ---

  // Sensoren initialisieren (Interrupts setzen)
  sensors[0].begin(interrupt_sensor0);
  sensors[1].begin(interrupt_sensor1);

  // Heartbeat starten
  heartbeat.start();

  // Test: große Nachricht senden (zeigt dir BLE-Payload-Limits)
  send_text_message(String(200, 'A'));
}

/* ============================================================================
   LOOP
   ----------------------------------------------------------------------------
   - Sensoren updaten (Master/Slave)
   - Heartbeat senden
   - Neue Messungen senden
   - Button verarbeiten
   ========================================================================== */
void loop() {
  // WICHTIG: slave wird als Referenz übergeben (nicht by-value!)
  sensors[0].update(true,  sensors[1]);  // Sensor0 = master, Sensor1 = slave
  sensors[1].update(false, sensors[0]);  // Sensor1 nicht master

  // Heartbeat alle 1s
  if (heartbeat.check()) {
    send_heartbeat();
    heartbeat.start();
  }

  // Messungen senden, wenn verfügbar
  for (uint8_t i = 0; i < sensors_length; i++) {
    Sensor& sensor = sensors[i];

    if (sensor.has_new_measurement) {
      const SensorMeasurement& measurement = sensor.measurement;

      // Default-Werte bei Timeout
      double distance = 99.0;
      uint32_t tof_ns = 10000;

      // Wenn kein Timeout: echte Werte berechnen
      if (!measurement.timeout) {
        distance = measurement.get_distance(); // Meter
        tof_ns = measurement.tof * 1000;      // µs -> ns (so wie dein Original)
      }

      send_distance_measurement(sensor.source_id, distance, tof_ns);
      sensor.has_new_measurement = false;
    }
  }

  // Button-Library updaten + Event senden
  button.handle();
  if (button.gotPressed()) {
    send_button_press();
  }

  // packetSerial.update(); // falls du USB PacketSerial wieder nutzen willst
}
