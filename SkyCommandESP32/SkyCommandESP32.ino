// =============================================================================
// SkyCommandESP32 — WebSocket-based roaster control for ESP32
//
// Migration from SkyCommand (AVR/Nano) to ESP32 over WebSocket.
// Follows the safety-first build order in ESP32_MIGRATION_PLAN.md.
//
// Build phase status:
//   [x] Phase 1 — RMT RX (ported from SkywalkerSpyESP32, validate first)
//   [x] Phase 2 — WebSocket server, Artisan connection (READ only)
//   [x] Phase 3 — Dual-core task split (roaster on Core 1, WiFi on Core 0)
//   [ ] Phase 4 — TX: shutdown / ESTOP         (TODO)
//   [ ] Phase 5 — TX: bean cooler              (TODO)
//   [ ] Phase 6 — TX: drum                     (TODO)
//   [ ] Phase 7 — TX: air / vent               (TODO)
//   [ ] Phase 8 — TX: heat / burner            (TODO — do last)
//
// SAFETY RULES — do not bypass:
//   - Do not enable TX phases until the corresponding RX validation is complete
//   - Do not raise MAX_TEMP_C above 300
//   - Do not disable the 10-second Artisan timeout
//   - Do not remove checksum validation
//   - Heat/burner (OT1) must be the last output enabled — see Phase 8 pre-conditions
//
// Artisan WebSocket config:
//   URL:  ws://<esp32-ip>:81
//   READ response format: "ET,BT" (e.g. "0.0,185.4")
//
// License: GPLv3
// =============================================================================

//#define __DEBUG__   // Verbose RMT decode and command logging
//#define __WARN__    // Checksum failures and retry warnings

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <driver/rmt.h>

// -----------------------------------------------------------------------------
// WiFi credentials — replace before flashing
// TODO (Phase 9): replace with WiFiManager captive portal
// -----------------------------------------------------------------------------
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// -----------------------------------------------------------------------------
// WebSocket server
// -----------------------------------------------------------------------------
WebSocketsServer webSocket(81);

// -----------------------------------------------------------------------------
// Pin definitions
// -----------------------------------------------------------------------------
const gpio_num_t RX_PIN = GPIO_NUM_4;   // Roaster → ESP32 (via 1kΩ/2kΩ divider)
const gpio_num_t TX_PIN = GPIO_NUM_5;   // ESP32 → Roaster (via 74HCT1G125)

// -----------------------------------------------------------------------------
// RMT channels
// -----------------------------------------------------------------------------
const rmt_channel_t RMT_RX_CHANNEL = RMT_CHANNEL_0;
const rmt_channel_t RMT_TX_CHANNEL = RMT_CHANNEL_1;

// -----------------------------------------------------------------------------
// Roaster protocol timing constants (microseconds)
// Identical to SkyCommand — do not change without re-validating on hardware
// -----------------------------------------------------------------------------
const int PREAMBLE_US        = 7500;   // TX preamble pulse duration
const int POST_PREAMBLE_US   = 3800;   // TX gap after preamble
const int BIT_ONE_US         = 1500;   // TX pulse duration for bit '1'
const int BIT_ZERO_US        = 650;    // TX pulse duration for bit '0'
const int INTER_BIT_US       = 750;    // TX gap between bits

const int PREAMBLE_MIN_US    = 6500;   // RX preamble lower bound
const int PREAMBLE_MAX_US    = 8500;   // RX preamble upper bound
const int BIT_THRESHOLD_US   = 1200;   // RX: above = '1', below = '0'
const int IDLE_THRESHOLD_US  = 9000;   // RMT RX idle gap (end of message)

const uint8_t RMT_CLK_DIV    = 80;    // 80 MHz APB / 80 = 1 µs per tick
const uint8_t RMT_FILTER_TICKS = 200; // Reject glitches < 200 µs

// -----------------------------------------------------------------------------
// Message buffer sizes and byte positions
// -----------------------------------------------------------------------------
const int ROASTER_MSG_LEN    = 7;
const int CONTROLLER_MSG_LEN = 6;

// Byte positions in the outgoing 6-byte controller message
const int BYTE_VENT   = 0;
const int BYTE_FILTER = 1;
const int BYTE_COOL   = 2;
const int BYTE_DRUM   = 3;
const int BYTE_HEAT   = 4;
const int BYTE_CHKSUM = 5;

uint8_t receiveBuffer[ROASTER_MSG_LEN];
uint8_t sendBuffer[CONTROLLER_MSG_LEN];   // Protected by s_mutex

// -----------------------------------------------------------------------------
// Safety limits
// -----------------------------------------------------------------------------
const int MAX_TEMP_C = 300;   // Emergency stop threshold — do not raise

// -----------------------------------------------------------------------------
// Artisan communication timeout
// If no valid READ or control command received within this window, shut down.
// -----------------------------------------------------------------------------
const unsigned long ARTISAN_TIMEOUT_US = 10000000UL;   // 10 seconds

// -----------------------------------------------------------------------------
// Shared state between Core 0 (WiFi/WebSocket) and Core 1 (roaster loop)
// Access must be protected by s_mutex.
// -----------------------------------------------------------------------------
typedef struct {
  double   tempC;                        // Latest temperature reading
  bool     eStopActive;                  // Latched emergency stop
  unsigned long lastArtisanEventTime;    // micros() of last valid Artisan command
} SharedState;

static SharedState   s_state  = { 0.0, false, 0 };
static SemaphoreHandle_t s_mutex = NULL;

// -----------------------------------------------------------------------------
// RMT RX ring buffer
// -----------------------------------------------------------------------------
static RingbufHandle_t s_rb = NULL;

// =============================================================================
// Checksum and send-buffer helpers
// =============================================================================
static void updateChecksum() {
  uint8_t sum = 0;
  for (int i = 0; i < CONTROLLER_MSG_LEN - 1; i++) sum += sendBuffer[i];
  sendBuffer[BYTE_CHKSUM] = sum;
}

static void setOutputByte(int index, uint8_t value) {
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    sendBuffer[index] = value;
    updateChecksum();
    xSemaphoreGive(s_mutex);
  }
}

static void doShutdown() {
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    memset(sendBuffer, 0, CONTROLLER_MSG_LEN);
    xSemaphoreGive(s_mutex);
  }
}

static void doEStop() {
  // Heat to 0, vent to 100 — mirrors SkyCommand::eStop()
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    sendBuffer[BYTE_HEAT] = 0;
    sendBuffer[BYTE_VENT] = 100;
    updateChecksum();
    s_state.eStopActive = true;
    xSemaphoreGive(s_mutex);
  }
}

// =============================================================================
// RMT initialisation
// =============================================================================
static void initRmtRx() {
  rmt_config_t cfg = {};
  cfg.rmt_mode                      = RMT_MODE_RX;
  cfg.channel                       = RMT_RX_CHANNEL;
  cfg.gpio_num                      = RX_PIN;
  cfg.clk_div                       = RMT_CLK_DIV;
  cfg.mem_block_num                 = 4;
  cfg.rx_config.idle_threshold      = IDLE_THRESHOLD_US;
  cfg.rx_config.filter_ticks_thresh = RMT_FILTER_TICKS;
  cfg.rx_config.filter_en           = true;

  ESP_ERROR_CHECK(rmt_config(&cfg));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_RX_CHANNEL, 1024, 0));
  rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &s_rb);
  rmt_rx_start(RMT_RX_CHANNEL, true);
}

static void initRmtTx() {
  // TODO (Phase 4): configure RMT TX channel
  // rmt_config_t cfg = {};
  // cfg.rmt_mode     = RMT_MODE_TX;
  // cfg.channel      = RMT_TX_CHANNEL;
  // cfg.gpio_num     = TX_PIN;
  // cfg.clk_div      = RMT_CLK_DIV;
  // cfg.mem_block_num = 2;
  // cfg.tx_config.idle_output_en    = true;
  // cfg.tx_config.idle_level        = RMT_IDLE_LEVEL_HIGH;
  // cfg.tx_config.carrier_en        = false;
  // cfg.tx_config.loop_en           = false;
  // ESP_ERROR_CHECK(rmt_config(&cfg));
  // ESP_ERROR_CHECK(rmt_driver_install(RMT_TX_CHANNEL, 0, 0));
}

// =============================================================================
// RMT TX — send controller message to roaster
// TODO (Phase 4): implement after RX is validated
// =============================================================================
static void sendRoasterMessage() {
  // TODO (Phase 4): build rmt_item32_t[] pulse sequence from sendBuffer
  // and transmit via rmt_write_items().
  //
  // Frame format (LSB first per byte):
  //   Preamble : 7500 µs LOW, 3800 µs HIGH
  //   Per bit  : (1500 or 650) µs LOW, 750 µs HIGH
  //
  // IMPORTANT: validate output on a logic analyzer before connecting
  // to the roaster. Do not skip this step.
}

// =============================================================================
// RMT RX — decode a captured frame into receiveBuffer
// Returns true on valid checksum.
// =============================================================================
static bool decodeRmtFrame(rmt_item32_t* items, size_t count) {
  if (count < 2) return false;

  uint32_t preambleDuration = items[0].duration0;
  if (preambleDuration < (uint32_t)PREAMBLE_MIN_US ||
      preambleDuration > (uint32_t)PREAMBLE_MAX_US) {
#ifdef __WARN__
    Serial.printf("[!] Bad preamble: %u µs\n", preambleDuration);
#endif
    return false;
  }

  const int DATA_BITS = ROASTER_MSG_LEN * 8;
  if ((int)count < DATA_BITS + 1) {
#ifdef __WARN__
    Serial.printf("[!] Short frame: %d items\n", (int)count);
#endif
    return false;
  }

  memset(receiveBuffer, 0, ROASTER_MSG_LEN);

  for (int i = 0; i < DATA_BITS; i++) {
    if (items[i + 1].duration0 > (uint32_t)BIT_THRESHOLD_US) {
      receiveBuffer[i / 8] |= (1 << (i % 8));
    }
  }

  uint8_t sum = 0;
  for (int i = 0; i < ROASTER_MSG_LEN - 1; i++) sum += receiveBuffer[i];

#ifdef __WARN__
  if (sum != receiveBuffer[ROASTER_MSG_LEN - 1]) {
    Serial.printf("[!] Checksum fail: computed 0x%02X, got 0x%02X\n",
                  sum, receiveBuffer[ROASTER_MSG_LEN - 1]);
  }
#endif

  return sum == receiveBuffer[ROASTER_MSG_LEN - 1];
}

// =============================================================================
// Temperature calculation — identical polynomial to SkyCommand
// Do not modify without re-running Data/model3.py and updating both sketches.
// =============================================================================
static double calculateTemp() {
  double x = ((receiveBuffer[0] << 8) | receiveBuffer[1]) / 1000.0;
  double y = ((receiveBuffer[2] << 8) | receiveBuffer[3]) / 1000.0;

  double v = 583.1509258523457
    + -714.0345395202813  * x
    + -196.071718077524   * y
    +  413.37964344228334 * x * x
    + 2238.149675349052   * x * y
    + -4099.91031297056   * y * y
    +  357.49007607425233 * x * x * x
    + -5001.419602972793  * x * x * y
    + 8242.08618555862    * x * y * y
    +  247.6124684730026  * y * y * y
    + -555.8643213534281  * x * x * x * x
    + 3879.431274654493   * x * x * x * y
    + -6885.682277959339  * x * x * y * y
    + 2868.4191998911865  * x * y * y * y
    + -1349.1588373011923 * y * y * y * y;

  return (v - 32.0) * 5.0 / 9.0;  // Return °C always
}

// =============================================================================
// Failsafe checks — run on Core 1 every roaster loop iteration
// =============================================================================
static void failsafeChecks() {
  unsigned long now = micros();
  bool timeout     = false;
  bool overTemp    = false;

  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    unsigned long elapsed = now - s_state.lastArtisanEventTime;
    timeout  = (elapsed > ARTISAN_TIMEOUT_US);
    overTemp = (s_state.tempC > MAX_TEMP_C);
    xSemaphoreGive(s_mutex);
  }

  if (timeout) {
#ifdef __WARN__
    Serial.println("[!] Artisan timeout — shutting down");
#endif
    doShutdown();
  }

  if (overTemp) {
#ifdef __WARN__
    Serial.println("[!] Over-temperature — emergency stop");
#endif
    doEStop();
  }
}

// =============================================================================
// Core 1 task — roaster protocol loop
// Runs independently of WiFi/WebSocket. Reads temperature continuously
// and transmits the current sendBuffer to keep the roaster alive.
// =============================================================================
static void roasterTask(void* /*params*/) {
  int failCount = 0;
  const int MAX_FAIL = 10;

  for (;;) {
    // --- TX (send controller message to roaster) ---
    // TODO (Phase 4): uncomment once TX is implemented and validated
    // sendRoasterMessage();

    // --- RX (read temperature from roaster) ---
    size_t rx_size = 0;
    rmt_item32_t* items = (rmt_item32_t*)xRingbufferReceive(
        s_rb, &rx_size, pdMS_TO_TICKS(500));

    if (items != NULL) {
      size_t count = rx_size / sizeof(rmt_item32_t);

      if (decodeRmtFrame(items, count)) {
        double t = calculateTemp();
        failCount = 0;

        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          s_state.tempC = t;
          xSemaphoreGive(s_mutex);
        }
      } else {
        failCount++;
        if (failCount > MAX_FAIL) failCount = MAX_FAIL;
#ifdef __WARN__
        Serial.printf("[!] RX decode fail (%d)\n", failCount);
#endif
      }

      vRingbufferReturnItem(s_rb, (void*)items);
    } else {
      // No frame received within timeout — roaster may be offline
#ifdef __WARN__
      Serial.println("[!] RX timeout — no roaster message");
#endif
    }

    // --- Failsafe ---
    failsafeChecks();
  }
}

// =============================================================================
// WebSocket event handler (Core 0)
// Parses Artisan commands and dispatches to shared state / sendBuffer.
//
// Control commands (OT1, OT2, DRUM, FILTER, COOL, OFF, ESTOP) return
// #READONLY until the corresponding TX phase is implemented and validated.
// Remove the #READONLY guard for each command only when its TX phase is done.
// =============================================================================
static void webSocketEvent(uint8_t num, WStype_t type,
                           uint8_t* payload, size_t /*length*/) {
  if (type != WStype_TEXT) return;

  String msg = String((char*)payload);
  msg.trim();

  int sep = msg.indexOf(';');
  String command = (sep >= 0) ? msg.substring(0, sep) : msg;
  int    value   = (sep >= 0) ? msg.substring(sep + 1).toInt() : 0;

#ifdef __DEBUG__
  Serial.printf("[WS] cmd='%s' val=%d\n", command.c_str(), value);
#endif

  if (command == "READ") {
    double t = 0.0;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      t = s_state.tempC;
      s_state.lastArtisanEventTime = micros();
      xSemaphoreGive(s_mutex);
    }
    // ET = 0.0 (not wired), BT = roaster temperature
    String response = "0.0," + String(t, 1);
    webSocket.sendTXT(num, response);

  } else if (command == "CHAN") {
    webSocket.sendTXT(num, "# Active channels set to 0200");

  } else if (command == "UNITS") {
    // Temperature units selection — stored for future use
    // Currently always returns °C
    webSocket.sendTXT(num, "#OK");

  } else if (command == "OFF") {
    // TODO (Phase 4): remove #READONLY and call doShutdown()
    webSocket.sendTXT(num, "#READONLY");

  } else if (command == "ESTOP") {
    // TODO (Phase 4): remove #READONLY and call doEStop()
    webSocket.sendTXT(num, "#READONLY");

  } else if (command == "COOL") {
    // TODO (Phase 5): remove #READONLY and call setOutputByte(BYTE_COOL, value)
    webSocket.sendTXT(num, "#READONLY");

  } else if (command == "DRUM") {
    // TODO (Phase 6): remove #READONLY
    //   uint8_t v = (value != 0) ? 100 : 0;
    //   setOutputByte(BYTE_DRUM, v);
    webSocket.sendTXT(num, "#READONLY");

  } else if (command == "OT2") {   // Vent / air
    // TODO (Phase 7): remove #READONLY and call setOutputByte(BYTE_VENT, value)
    webSocket.sendTXT(num, "#READONLY");

  } else if (command == "OT1") {   // Heat / burner — enable last
    // TODO (Phase 8): remove #READONLY and call setOutputByte(BYTE_HEAT, value)
    // Pre-conditions in ESP32_MIGRATION_PLAN.md Phase 8 must ALL be met first.
    webSocket.sendTXT(num, "#READONLY");

  } else if (command == "FILTER") {
    // TODO (Phase 5 or later): remove #READONLY
    webSocket.sendTXT(num, "#READONLY");

  } else {
#ifdef __WARN__
    Serial.printf("[!] Unknown command: %s\n", command.c_str());
#endif
    webSocket.sendTXT(num, "#UNKNOWN");
  }
}

// =============================================================================
// Setup (Core 0)
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("SkyCommandESP32 starting...");

  // Initialise pins
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);   // Idle HIGH (line is active-LOW protocol)

  // Shared state
  s_mutex = xSemaphoreCreateMutex();
  s_state.lastArtisanEventTime = micros();
  doShutdown();

  // RMT
  initRmtRx();
  // initRmtTx();  // TODO (Phase 4): uncomment when TX is ready

  // WiFi
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected. IP: %s\n", WiFi.localIP().toString().c_str());

  // WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.printf("WebSocket server started on ws://%s:81\n",
                WiFi.localIP().toString().c_str());

  // Roaster task pinned to Core 1
  xTaskCreatePinnedToCore(
    roasterTask,    // function
    "roasterTask",  // name
    8192,           // stack size (bytes)
    NULL,           // parameters
    2,              // priority
    NULL,           // task handle (not needed)
    1               // Core 1
  );

  Serial.println("Setup complete. Artisan: READ-only mode until TX phases complete.");
}

// =============================================================================
// Loop (Core 0) — WebSocket polling only
// All roaster protocol work runs on Core 1 via roasterTask.
// =============================================================================
void loop() {
  webSocket.loop();
}
