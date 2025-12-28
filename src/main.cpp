#include <Arduino.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <utils/button.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <HardwareSerial.h>
#include <esp_timer.h>   // esp_timer_get_time(): 64-bit uptime (µs since boot)

#include "openbikesensor.pb.h"

// ============================================================================
// OBS Lite LiDAR Firmware (clean)
// - Sends OpenBikeSensor protobuf Events via BLE notify (Nordic UART style)
// - Time: monotonic uptime -> reference = ARBITRARY (no fake UNIX time)
// - TF-Luna on UART provides LEFT distance
// - RIGHT distance is a constant dummy value
// - Heartbeat: 1 Hz
// - Distance: 10 Hz
// - Button: sends UserInput event
// ============================================================================

// ------------------------- Device info strings -------------------------
static constexpr const char* DEV_LOCAL_NAME   = "OBS Lite LiDAR";
static constexpr const char* DEV_MANUFACTURER = "OpenBikeSensor";
static constexpr const char* DEV_FW_REV       = "OBS-Lite-LiDAR";

// ------------------------- BLE UUIDs (OBS Lite / Nordic UART style) ----
static constexpr const char* OBS_BLE_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char* OBS_BLE_CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

// ------------------------- Hardware pins -------------------------------
static constexpr int PUSHBUTTON_PIN = 2;

static constexpr int TF_LUNA_RX_PIN = 16;
static constexpr int TF_LUNA_TX_PIN = 17;

// ------------------------- Protobuf buffer -----------------------------
static constexpr size_t PB_BUFFER_SIZE = 1024;
static uint8_t      pb_buffer[PB_BUFFER_SIZE];
static pb_ostream_t pb_ostream;

// ============================================================================
// Time (Uptime only) correct semantic: ARBITRARY
// ============================================================================
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
    Serial.println("BLE: connected");
  }
  void onDisconnect(BLEServer* s) override {
    g_bleConnected = false;
    Serial.println("BLE: disconnected -> advertising");
    s->startAdvertising();
  }
};

// ============================================================================
// Button
// ============================================================================
Button button(PUSHBUTTON_PIN);

// ============================================================================
// BLE send helper
// - Sends one protobuf Event as a single BLE notify.
// - delay(1) gives BLE stack a tiny bit of time under frequent notifications.
// ============================================================================
static uint32_t g_last_send_len = 0;

static inline void send_event_bytes(const uint8_t* data, size_t len) {
  g_last_send_len = (uint32_t)len;

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

// ============================================================================
// TF-Luna UART
// ============================================================================
HardwareSerial TFSerial(2);

volatile bool    lidar_has_value  = false;
float            lidar_distance_m = -1.0f;
uint16_t         lidar_strength   = 0;
float            lidar_temp_c     = 0.0f;

// quality thresholds
static constexpr uint16_t AMP_MIN_VALID  = 100;
static constexpr uint16_t AMP_OVEREXPOSE = 0xFFFF;

// mark sensor stale if no frames for too long
static constexpr uint32_t LIDAR_STALE_MS = 600;

// rate-limit warnings
static constexpr uint32_t WARN_COOLDOWN_MS = 30000;
static uint32_t lastWarnMs  = 0;
static uint32_t lastFrameMs = 0;

// TF-Luna frame: 0x59 0x59 + 7 bytes + checksum = 9 bytes
static bool readTfLunaFrame(float& distance_m, uint16_t& strength, float& temp_c) {
  static uint8_t buf[9];
  static uint8_t idx = 0;

  while (TFSerial.available()) {
    const uint8_t b = (uint8_t)TFSerial.read();

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
      if (((uint8_t)sum) != buf[8]) return false;

      const uint16_t dist_cm = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
      strength               = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
      const uint16_t tempRaw = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
      temp_c = ((float)tempRaw) / 8.0f - 256.0f;

      if (strength < AMP_MIN_VALID || strength == AMP_OVEREXPOSE ||
          dist_cm == 0 || dist_cm == 0xFFFF) {
        distance_m = -1.0f;
      } else {
        distance_m = (float)dist_cm / 100.0f;
      }
      return true;
    }
  }
  return false;
}

// ============================================================================
// Dummy right sensor (constant)
// ============================================================================
static constexpr float DUMMY_RIGHT_METERS = 2.33f;

// status print 1x/s
static uint32_t lastStatusMs = 0;

// ============================================================================
// setup()
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // ---- BLE init ----
  BLEDevice::init(DEV_LOCAL_NAME);

  g_bleServer = BLEDevice::createServer();
  g_bleServer->setCallbacks(new ObsBleServerCallbacks());

  // Lite service + TX notify characteristic
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
    BLEUUID((uint16_t)0x2A26), // Firmware Revision String
    BLECharacteristic::PROPERTY_READ
  );
  fwRev->setValue(DEV_FW_REV);

  BLECharacteristic* manu = devInfo->createCharacteristic(
    BLEUUID((uint16_t)0x2A29), // Manufacturer Name String
    BLECharacteristic::PROPERTY_READ
  );
  manu->setValue(DEV_MANUFACTURER);

  devInfo->start();

  // Advertising: include service UUID + scan response with name
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(OBS_BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  // ---- TF-Luna UART ----
  TFSerial.begin(115200, SERIAL_8N1, TF_LUNA_RX_PIN, TF_LUNA_TX_PIN);

  heartbeat.start();
  lidarSendTimer.start();

  // Sent only if connected
  send_text_message(String("Hello: ") + DEV_LOCAL_NAME);
}

// ============================================================================
// loop()
// ============================================================================
void loop() {
  // 1) Read a few UART frames per loop (avoid spending too much time here)
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
    lastFrameMs      = (uint32_t)millis();
  }

  // 2) stale detection (no frames for a while)
  const uint32_t nowMs = (uint32_t)millis();
  if (lidar_has_value && (uint32_t)(nowMs - lastFrameMs) > LIDAR_STALE_MS) {
    lidar_has_value   = false;
    lidar_distance_m  = -1.0f;
    lidar_strength    = 0;

    // rate-limited warning
    if ((uint32_t)(nowMs - lastWarnMs) > WARN_COOLDOWN_MS) {
      send_text_message("LiDAR: no frames (UART timeout)", openbikesensor_TextMessage_Type_WARNING);
      lastWarnMs = nowMs;
    }
  }

  // 3) Heartbeat
  if (heartbeat.check()) {
    send_heartbeat();
    heartbeat.start();
  }

  // 4) Send LiDAR at fixed rate + dummy right constant
  if (lidarSendTimer.check()) {
    // importer-safe: always send; invalid -> sentinel large value
    const float leftMeters =
      (lidar_has_value && lidar_distance_m > 0.0f) ? lidar_distance_m : 99.0f;

    send_distance_measurement(1, leftMeters, 0);
    send_distance_measurement(2, DUMMY_RIGHT_METERS, 0);

    lidarSendTimer.start();
  }

  // 5) Button
  button.handle();
  if (button.gotPressed()) {
    send_button_press();
  }

  // 6) Status 1x/s (Serial)
  if ((uint32_t)(nowMs - lastStatusMs) >= 1000) {
    lastStatusMs = nowMs;
    Serial.printf("Status: BLE=%s last_len=%lu LiDAR=%.2fm Amp=%u Temp=%.1fC\n",
                  g_bleConnected ? "ON" : "OFF",
                  (unsigned long)g_last_send_len,
                  (double)lidar_distance_m,
                  (unsigned)lidar_strength,
                  (double)lidar_temp_c);
  }
}
