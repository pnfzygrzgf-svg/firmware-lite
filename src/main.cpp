#include <Arduino.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <utils/button.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <HardwareSerial.h>
#include <esp_timer.h>

#include <PacketSerial.h>

#include "openbikesensor.pb.h"

// ============================================================================
// OBS Lite LiDAR Firmware (2x TF-Luna)
// - Sends OpenBikeSensor protobuf Events via:
//     (A) BLE notify (Nordic UART style)  <-- unchanged for iOS app
//     (B) USB cable (PacketSerial / SLIP framing) for Android USB-Serial
//
// IMPORTANT (USB "connected" detection):
// - On ESP32-WROOM DevKit (USB-to-UART) we cannot reliably detect "port opened"
//   without DTR/RTS support OR a handshake from the host.
// - Therefore default is: USB ALWAYS SENDS (works with read-only Android apps).
// - If you can make Android send 1 byte on connect, set USB_REQUIRE_KNOCK = 1.
//
// VERSION: Fixed sentinel value issue - only valid measurements are sent
// ============================================================================

// ------------------------- USB behavior switch -------------------------
// 0 = USB always sends (recommended / works with read-only Android apps)
// 1 = USB sends only after any incoming byte/packet from host ("knock")
#define USB_REQUIRE_KNOCK 0

// ------------------------- Device info strings -------------------------
static constexpr const char* DEV_LOCAL_NAME   = "OBS Lite LiDAR";
static constexpr const char* DEV_MANUFACTURER = "OpenBikeSensor";
static constexpr const char* DEV_FW_REV       = "OBS-Lite-LiDAR-v2";

// ------------------------- BLE UUIDs (OBS Lite / Nordic UART style) ----
static constexpr const char* OBS_BLE_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char* OBS_BLE_CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

// ------------------------- Hardware pins -------------------------------
static constexpr int PUSHBUTTON_PIN = 25; // Button between GPIO25 and GND
static constexpr int LED_PIN = 2;         // DevKit onboard LED (GPIO2)

// LEFT (SensorL1): Sensor TX->26, Sensor RX->27 => ESP RX=26, TX=27
static constexpr int TF_LEFT_RX_PIN  = 26; // ESP RX  <- Sensor TX
static constexpr int TF_LEFT_TX_PIN  = 27; // ESP TX  -> Sensor RX

// RIGHT (SensorR1): Sensor TX->16, Sensor RX->17 => ESP RX=16, TX=17
static constexpr int TF_RIGHT_RX_PIN = 16; // ESP RX  <- Sensor TX
static constexpr int TF_RIGHT_TX_PIN = 17; // ESP TX  -> Sensor RX

// ------------------------- Protobuf buffer -----------------------------
static constexpr size_t PB_BUFFER_SIZE = 1024;
static uint8_t      pb_buffer[PB_BUFFER_SIZE];
static pb_ostream_t pb_ostream;

// ------------------------- USB cable output (PacketSerial) -------------
PacketSerial packetSerial; // uses Serial internally (USB-UART)

static bool g_usbHostConnected = false;

static void onSerialPacket(const uint8_t* /*buffer*/, size_t /*size*/) {
  // Any received packet marks host as connected (only used when USB_REQUIRE_KNOCK=1)
  g_usbHostConnected = true;
}

// ============================================================================
// Time (Uptime only) correct semantic: ARBITRARY
// ============================================================================
static inline openbikesensor_Time make_obs_time(int32_t time_source_id = 1) {
  const uint64_t us = (uint64_t)esp_timer_get_time();

  openbikesensor_Time t = openbikesensor_Time_init_zero;
  t.source_id   = time_source_id;
  t.reference   = openbikesensor_Time_Reference_ARBITRARY;
  t.seconds     = (int64_t)(us / 1000000ULL);
  t.nanoseconds = (int32_t)((us % 1000000ULL) * 1000ULL);
  return t;
}
static inline openbikesensor_Time make_cpu_time() { return make_obs_time(1); }

// ============================================================================
// BLE state
// ============================================================================
BLEServer*         g_bleServer  = nullptr;
BLECharacteristic* g_bleTxChar  = nullptr;
volatile bool      g_bleConnected = false;

class ObsBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    (void)s;
    g_bleConnected = true;
    // no Serial prints (PacketSerial stream!)
  }
  void onDisconnect(BLEServer* s) override {
    g_bleConnected = false;
    // no Serial prints (PacketSerial stream!)
    s->startAdvertising();
  }
};

// ============================================================================
// Button
// ============================================================================
Button button(PUSHBUTTON_PIN);

// ============================================================================
// Dual transport send helper (BLE + USB cable)
// ============================================================================
static uint32_t g_last_send_len = 0;

static inline void send_event_bytes(const uint8_t* data, size_t len) {
  g_last_send_len = (uint32_t)len;

  // USB
#if USB_REQUIRE_KNOCK
  if (g_usbHostConnected) {
    packetSerial.send(data, len);
  }
#else
  packetSerial.send(data, len);
#endif

  // BLE (UNCHANGED)
  if (g_bleConnected && g_bleTxChar != nullptr) {
    g_bleTxChar->setValue((uint8_t*)data, len);
    g_bleTxChar->notify();
    delay(1);
  }
}

// ============================================================================
// Nanopb string callback helper (needed for TextMessage)
// ============================================================================
static bool _write_string(pb_ostream_t* stream, const pb_field_iter_t* field, void* const* arg) {
  String& str = *((String*)(*arg));
  if (!pb_encode_tag_for_field(stream, field)) return false;
  return pb_encode_string(stream, (uint8_t*)str.c_str(), str.length());
}
static inline void write_string(pb_callback_t& target, String& str) {
  target.arg = &str;
  target.funcs.encode = &_write_string;
}

// ============================================================================
// Encode + send (timestamp is always applied here)
// ============================================================================
static inline bool encode_and_send(openbikesensor_Event& event) {
  event.time_count = 1;
  event.time[0] = make_cpu_time();

  pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
  const bool ok = pb_encode(&pb_ostream, &openbikesensor_Event_msg, &event);
  if (!ok) return false;
  if (pb_ostream.bytes_written == 0) return false;

  send_event_bytes(pb_buffer, pb_ostream.bytes_written);
  return true;
}

// ============================================================================
// OBS event helpers
// ============================================================================
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

// ============================================================================
// Wrap-safe Timer (millis overflow safe)
// ============================================================================
class Timer {
public:
  explicit Timer(uint32_t delay_ms) : delay(delay_ms) {}
  void start() { trigger_at = (uint32_t)millis() + delay; }
  bool check() {
    if (trigger_at == 0) return false;
    const uint32_t now = (uint32_t)millis();
    if ((int32_t)(now - trigger_at) >= 0) {
      trigger_at = 0;
      return true;
    }
    return false;
  }
private:
  uint32_t trigger_at = 0;
  uint32_t delay;
};

Timer heartbeat(1000);      // 1 Hz
Timer lidarSendTimer(100);  // 10 Hz
Timer ledOffTimer(150);

// ============================================================================
// TF-Luna parsing (state per sensor!)
// ============================================================================
static constexpr uint16_t AMP_MIN_VALID  = 100;
static constexpr uint16_t AMP_OVEREXPOSE = 0xFFFF;

static constexpr uint32_t LIDAR_STALE_MS = 600;
static constexpr uint32_t WARN_COOLDOWN_MS = 30000;

// --- Left-side plausibility + "hold last good value" behavior ---
static constexpr float    LEFT_MAX_METERS   = 4.0f;
// How long we keep sending last valid LEFT value after last received frame.
static constexpr uint32_t LEFT_HOLD_MS      = 2000;

// --- General validity range for all sensors ---
static constexpr float    MIN_VALID_METERS  = 0.02f;  // 2cm minimum
static constexpr float    MAX_VALID_METERS  = 10.0f;  // 10m maximum

// --- Sensor IDs (avoid hardcoded magic numbers) ---
static constexpr uint32_t LEFT_SENSOR_ID  = 1;
static constexpr uint32_t RIGHT_SENSOR_ID = 2;

struct TfLunaParser {
  uint8_t buf[9];
  uint8_t idx = 0;
};

enum TfLunaResult : uint8_t {
  TF_NO_DATA      = 0,
  TF_VALID_FRAME  = 1,
  TF_BAD_CHECKSUM = 2
};

static TfLunaResult readTfLunaFrame(HardwareSerial& port, TfLunaParser& p,
                                   float& distance_m, uint16_t& strength, float& temp_c) {
  while (port.available()) {
    const uint8_t b = (uint8_t)port.read();

    if (p.idx == 0) {
      if (b != 0x59) continue;
      p.buf[p.idx++] = b;
      continue;
    }
    if (p.idx == 1) {
      if (b != 0x59) { p.idx = 0; continue; }
      p.buf[p.idx++] = b;
      continue;
    }

    p.buf[p.idx++] = b;

    if (p.idx == 9) {
      p.idx = 0;

      uint16_t sum = 0;
      for (int i = 0; i < 8; i++) sum += p.buf[i];
      if (((uint8_t)sum) != p.buf[8]) {
        return TF_BAD_CHECKSUM;
      }

      const uint16_t dist_cm = (uint16_t)p.buf[2] | ((uint16_t)p.buf[3] << 8);
      strength               = (uint16_t)p.buf[4] | ((uint16_t)p.buf[5] << 8);
      const uint16_t tempRaw = (uint16_t)p.buf[6] | ((uint16_t)p.buf[7] << 8);
      temp_c = ((float)tempRaw) / 8.0f - 256.0f;

      if (strength < AMP_MIN_VALID || strength == AMP_OVEREXPOSE ||
          dist_cm == 0 || dist_cm == 0xFFFF) {
        distance_m = -1.0f;
      } else {
        distance_m = (float)dist_cm / 100.0f;
      }
      return TF_VALID_FRAME;
    }
  }
  return TF_NO_DATA;
}

// ============================================================================
// Two-sensor wrapper
// ============================================================================
HardwareSerial TFLeft(2);   // UART2
HardwareSerial TFRight(1);  // UART1

struct TfLunaSensor {
  const char*      name;
  uint32_t         source_id;

  HardwareSerial*  port;
  int              rx_pin;
  int              tx_pin;

  TfLunaParser     parser;

  bool             has_value = false;
  float            distance_m = -1.0f;
  uint16_t         strength = 0;
  float            temp_c = 0.0f;

  // Last known good value (for holding)
  float            last_good_distance_m = -1.0f;
  uint32_t         lastGoodMs = 0;

  uint32_t         lastFrameMs = 0;
  uint32_t         lastWarnMs  = 0;

  TfLunaSensor(const char* n, uint32_t sid, HardwareSerial* p, int rx, int tx)
  : name(n), source_id(sid), port(p), rx_pin(rx), tx_pin(tx) {
    parser.idx = 0;
  }
};

enum { IDX_LEFT = 0, IDX_RIGHT = 1 };

// ============================================================================
// VARIANTE A: LEFT/RIGHT LOGISCH TAUSCHEN
// - LEFT nutzt jetzt den "rechten" UART/Pins
// - RIGHT nutzt jetzt den "linken" UART/Pins
// ============================================================================
TfLunaSensor sensors[2] = {
  TfLunaSensor("LEFT",  LEFT_SENSOR_ID,  &TFRight, TF_RIGHT_RX_PIN, TF_RIGHT_TX_PIN),
  TfLunaSensor("RIGHT", RIGHT_SENSOR_ID, &TFLeft,  TF_LEFT_RX_PIN,  TF_LEFT_TX_PIN)
};

static inline void poll_sensor(TfLunaSensor& s, uint32_t nowMs) {
  float d_m;
  uint16_t amp;
  float t_c;

  int validFrames = 0;
  int badChecksums = 0;

  int completedFrames = 0;
  static constexpr int MAX_COMPLETED_FRAMES_PER_POLL = 10;

  while (validFrames < 3 && completedFrames < MAX_COMPLETED_FRAMES_PER_POLL) {
    TfLunaResult r = readTfLunaFrame(*s.port, s.parser, d_m, amp, t_c);
    if (r == TF_NO_DATA) break;

    completedFrames++;

    if (r == TF_BAD_CHECKSUM) {
      badChecksums++;
      continue;
    }

    validFrames++;

    s.distance_m  = d_m;
    s.strength    = amp;
    s.temp_c      = t_c;
    s.has_value   = true;
    s.lastFrameMs = nowMs;

    // Update last_good_distance_m only if value is plausible
    if (s.distance_m >= MIN_VALID_METERS) {
      // Left side: ignore >4m as outlier; right side: accept all valid
      if (s.source_id != LEFT_SENSOR_ID || s.distance_m <= LEFT_MAX_METERS) {
        s.last_good_distance_m = s.distance_m;
        s.lastGoodMs = nowMs;
      }
    }
  }

  if (badChecksums >= 3 && (uint32_t)(nowMs - s.lastWarnMs) > WARN_COOLDOWN_MS) {
    char msgBuf[64];
    snprintf(msgBuf, sizeof(msgBuf), "LiDAR %s: %d checksum errors", s.name, badChecksums);
    send_text_message(String(msgBuf), openbikesensor_TextMessage_Type_WARNING);
    s.lastWarnMs = nowMs;
  }

  if (s.has_value && (uint32_t)(nowMs - s.lastFrameMs) > LIDAR_STALE_MS) {
    s.has_value  = false;
    s.distance_m = -1.0f;
    s.strength   = 0;

    if ((uint32_t)(nowMs - s.lastWarnMs) > WARN_COOLDOWN_MS) {
      char msgBuf[64];
      snprintf(msgBuf, sizeof(msgBuf), "LiDAR %s: no frames (UART timeout)", s.name);
      send_text_message(String(msgBuf), openbikesensor_TextMessage_Type_WARNING);
      s.lastWarnMs = nowMs;
    }
  }
}

// ============================================================================
// Get valid distance or negative value (= invalid, don't send)
//
// Returns:
//   > 0.0f  : Valid distance in meters -> send event
//   <= 0.0f : Invalid/no measurement   -> DO NOT send event
// ============================================================================
static inline float get_valid_distance_m(const TfLunaSensor& s, uint32_t nowMs) {
  // Left sensor: special handling with hold-last-good and plausibility check
  if (s.source_id == LEFT_SENSOR_ID) {
    const bool have_current = (s.has_value && s.distance_m >= MIN_VALID_METERS && s.distance_m <= MAX_VALID_METERS);
    const bool current_plausible = (have_current && s.distance_m <= LEFT_MAX_METERS);

    if (current_plausible) {
      return s.distance_m;
    }

    // Hold last good value if recent enough
    if (s.last_good_distance_m >= MIN_VALID_METERS && (uint32_t)(nowMs - s.lastGoodMs) <= LEFT_HOLD_MS) {
      return s.last_good_distance_m;
    }

    // Invalid -> return negative to signal "don't send"
    return -1.0f;
  }

  // Right sensor: validity check with range limits
  if (s.has_value && s.distance_m >= MIN_VALID_METERS && s.distance_m <= MAX_VALID_METERS) {
    return s.distance_m;
  }

  // Invalid -> return negative to signal "don't send"
  return -1.0f;
}

// ============================================================================
// Check if distance is within valid range for sending
// ============================================================================
static inline bool is_valid_for_sending(float distance_m) {
  return (distance_m >= MIN_VALID_METERS && distance_m <= MAX_VALID_METERS);
}

static bool ledIsOn = false;

// ============================================================================
// setup()
// ============================================================================
void setup() {
  packetSerial.setPacketHandler(&onSerialPacket);
  packetSerial.begin(115200);
  delay(200);

#if USB_REQUIRE_KNOCK
  g_usbHostConnected = false;
#else
  g_usbHostConnected = true;   // ensures Android read-only apps work
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(PUSHBUTTON_PIN, INPUT_PULLUP);

  // ---- BLE init ---- (UNCHANGED)
  BLEDevice::init(DEV_LOCAL_NAME);

  g_bleServer = BLEDevice::createServer();
  static ObsBleServerCallbacks bleCallbacks;
  g_bleServer->setCallbacks(&bleCallbacks);

  BLEService* obsService = g_bleServer->createService(OBS_BLE_SERVICE_UUID);

  g_bleTxChar = obsService->createCharacteristic(
    OBS_BLE_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  g_bleTxChar->addDescriptor(new BLE2902());
  obsService->start();

  // Device Information Service (0x180A)
  BLEService* devInfo = g_bleServer->createService(BLEUUID((uint16_t)0x180A));

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

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(OBS_BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  // ---- TF-Luna UARTs ----
  for (auto& s : sensors) {
    s.port->begin(115200, SERIAL_8N1, s.rx_pin, s.tx_pin);
    s.lastFrameMs = (uint32_t)millis();  // Runtime-dependent, needs millis()
    s.parser.idx  = 0;                   // Reset parser state
  }

  heartbeat.start();
  lidarSendTimer.start();

  send_text_message(String("Hello: ") + DEV_LOCAL_NAME);
}

// ============================================================================
// loop()
// ============================================================================
void loop() {
  const uint32_t nowMs = (uint32_t)millis();

  // Keep PacketSerial RX state machine running
  packetSerial.update();

  // 1) Poll both sensors
  for (auto& s : sensors) {
    poll_sensor(s, nowMs);
  }

  // 2) Heartbeat
  if (heartbeat.check()) {
    send_heartbeat();
    heartbeat.start();
  }

  // 3) Send distances at fixed rate - ONLY VALID VALUES
  if (lidarSendTimer.check()) {
    for (const auto& s : sensors) {
      const float meters = get_valid_distance_m(s, nowMs);
      
      // Only send if value is valid and within reasonable range
      if (is_valid_for_sending(meters)) {
        send_distance_measurement(s.source_id, meters, 0);
      }
      // If invalid: simply don't send anything - no sentinel value!
    }
    lidarSendTimer.start();
  }

  // 4) Button (non-blocking LED)
  button.handle();
  if (button.gotPressed()) {
    digitalWrite(LED_PIN, HIGH);
    ledIsOn = true;
    ledOffTimer.start();
    send_button_press();
  }
  if (ledIsOn && ledOffTimer.check()) {
    digitalWrite(LED_PIN, LOW);
    ledIsOn = false;
  }
}
