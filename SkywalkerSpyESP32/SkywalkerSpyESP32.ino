// =============================================================================
// SkywalkerSpyESP32 — Phase 1: Passive RX spy (no TX to roaster)
//
// ESP32 port of SkywalkerSpy. Listens to the roaster's outgoing message,
// decodes temperature via the same polynomial used in SkyCommand, and
// outputs over USB serial for validation against the known-good Nano.
//
// SAFETY: This sketch never transmits to the roaster. GPIO 5 (TX) is
// intentionally left unconfigured. Do not add TX logic here.
//
// Validation target: run alongside the existing Nano/SkywalkerSpy and
// compare readings. Values must agree within 1°C across a full heat
// cycle before proceeding to Phase 2.
//
// License: GPLv3
// =============================================================================

//#define __DEBUG__   // Uncomment for verbose RMT decode output
//#define __WARN__    // Uncomment for checksum failure warnings

#include <driver/rmt.h>

// -----------------------------------------------------------------------------
// Pin definitions
// -----------------------------------------------------------------------------
const gpio_num_t RX_PIN = GPIO_NUM_4;   // Roaster → ESP32 (via 1kΩ/2kΩ divider)
// TX_PIN intentionally not defined — Phase 1 is receive-only

// -----------------------------------------------------------------------------
// RMT channel assignment
// -----------------------------------------------------------------------------
const rmt_channel_t RMT_RX_CHANNEL = RMT_CHANNEL_0;

// -----------------------------------------------------------------------------
// Roaster protocol timing constants (microseconds)
// Identical to SkyCommand/SkywalkerSpy — do not change without re-validating
// against hardware.
// -----------------------------------------------------------------------------
const int PREAMBLE_MIN_US    = 6500;   // Preamble lower bound  (~7.0ms nominal)
const int PREAMBLE_MAX_US    = 8500;   // Preamble upper bound  (~7.5ms nominal)
const int BIT_THRESHOLD_US   = 1200;   // Above = bit '1', below = bit '0'
const int IDLE_THRESHOLD_US  = 9000;   // RMT idle gap marking end of message

// RMT clock: APB @ 80 MHz, clk_div = 80 → 1 tick = 1 µs
const uint8_t RMT_CLK_DIV = 80;

// Glitch filter: reject pulses shorter than this many ticks (max 255)
// 200 µs is safe — valid bits are ≥ 650 µs
const uint8_t RMT_FILTER_TICKS = 200;

// -----------------------------------------------------------------------------
// Message buffer sizes
// -----------------------------------------------------------------------------
const int ROASTER_MSG_LEN = 7;   // Bytes in roaster → controller message

uint8_t receiveBuffer[ROASTER_MSG_LEN];

// -----------------------------------------------------------------------------
// RMT ring buffer handle
// -----------------------------------------------------------------------------
static RingbufHandle_t s_rb = NULL;

// -----------------------------------------------------------------------------
// Temperature state
// -----------------------------------------------------------------------------
double tempC = 0.0;

// -----------------------------------------------------------------------------
// RX failure tracking
// -----------------------------------------------------------------------------
bool failedToRead        = false;
int  readAttempts        = 0;
const int MAX_ATTEMPTS   = 10;

// =============================================================================
// RMT initialisation
// =============================================================================
void initRmtRx() {
  rmt_config_t cfg = {};
  cfg.rmt_mode                  = RMT_MODE_RX;
  cfg.channel                   = RMT_RX_CHANNEL;
  cfg.gpio_num                  = RX_PIN;
  cfg.clk_div                   = RMT_CLK_DIV;
  cfg.mem_block_num             = 4;            // 4 × 64 items = 256; plenty for 7 bytes
  cfg.rx_config.idle_threshold  = IDLE_THRESHOLD_US;
  cfg.rx_config.filter_ticks_thresh = RMT_FILTER_TICKS;
  cfg.rx_config.filter_en       = true;

  ESP_ERROR_CHECK(rmt_config(&cfg));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_RX_CHANNEL, 1024, 0));
  rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &s_rb);
  rmt_rx_start(RMT_RX_CHANNEL, true);
}

// =============================================================================
// Decode RMT items into receiveBuffer
//
// Each rmt_item32_t from an RX capture represents one pulse:
//   item.duration0 = LOW duration (the pulse itself)
//   item.duration1 = HIGH duration (the gap before the next pulse)
//
// Frame structure:
//   Item 0           : preamble  (duration0 ≈ 7500 µs)
//   Items 1..56      : data bits (LSB first, 8 bits × 7 bytes)
//
// Returns true if a valid frame was decoded and checksum passed.
// =============================================================================
bool decodeRmtFrame(rmt_item32_t* items, size_t count) {
  if (count < 2) {
    // Need at least preamble + 1 data item
    failedToRead = true;
    return false;
  }

  // Validate preamble (item 0)
  uint32_t preambleDuration = items[0].duration0;
  if (preambleDuration < PREAMBLE_MIN_US || preambleDuration > PREAMBLE_MAX_US) {
#ifdef __WARN__
    Serial.printf("[!] Bad preamble: %u µs\n", preambleDuration);
#endif
    failedToRead = true;
    return false;
  }

  // We expect 56 data items (7 bytes × 8 bits) after the preamble
  const int DATA_BITS = ROASTER_MSG_LEN * 8;
  if ((int)count < DATA_BITS + 1) {
#ifdef __WARN__
    Serial.printf("[!] Short frame: %d items (need %d)\n", (int)count, DATA_BITS + 1);
#endif
    failedToRead = true;
    return false;
  }

  memset(receiveBuffer, 0, ROASTER_MSG_LEN);

  for (int i = 0; i < DATA_BITS; i++) {
    rmt_item32_t item = items[i + 1]; // +1 to skip preamble
    uint32_t duration = item.duration0;

    if (duration > BIT_THRESHOLD_US) {
      // Bit is '1' — set bit (i % 8) of byte (i / 8), LSB first
      receiveBuffer[i / 8] |= (1 << (i % 8));
    }

#ifdef __DEBUG__
    Serial.printf("bit[%02d] dur=%u -> %d\n", i, duration, duration > BIT_THRESHOLD_US ? 1 : 0);
#endif
  }

  // Validate checksum (simple byte sum of first 6 bytes)
  uint8_t sum = 0;
  for (int i = 0; i < ROASTER_MSG_LEN - 1; i++) {
    sum += receiveBuffer[i];
  }

  bool ok = (sum == receiveBuffer[ROASTER_MSG_LEN - 1]);

#ifdef __WARN__
  if (!ok) {
    Serial.printf("[!] Checksum fail: computed 0x%02X, got 0x%02X\n",
                  sum, receiveBuffer[ROASTER_MSG_LEN - 1]);
  }
#endif

  failedToRead = !ok;
  return ok;
}

// =============================================================================
// Temperature calculation
// Identical polynomial to SkyCommand — do not modify without re-running
// Data/model3.py and updating both sketches together.
// =============================================================================
double calculateTemp() {
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

  // v is in °F — convert to °C
  tempC = (v - 32.0) * 5.0 / 9.0;
  return tempC;
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("SkywalkerSpyESP32 starting...");
  Serial.printf("RX GPIO: %d\n", (int)RX_PIN);

  pinMode(RX_PIN, INPUT);
  initRmtRx();

  Serial.println("RMT RX ready. Listening for roaster messages.");
  Serial.println("Output format: TEMP_C,RAW_A,RAW_B");
}

// =============================================================================
// Loop
// =============================================================================
void loop() {
  size_t rx_size = 0;

  // Block for up to 500 ms waiting for a complete RMT frame
  rmt_item32_t* items = (rmt_item32_t*)xRingbufferReceive(s_rb, &rx_size, pdMS_TO_TICKS(500));

  if (items == NULL) {
#ifdef __WARN__
    Serial.println("[!] No roaster message received within timeout");
#endif
    return;
  }

  size_t itemCount = rx_size / sizeof(rmt_item32_t);

#ifdef __DEBUG__
  Serial.printf("Frame received: %d items\n", (int)itemCount);
#endif

  if (decodeRmtFrame(items, itemCount)) {
    readAttempts = 0;
    calculateTemp();

    uint16_t rawA = ((uint16_t)receiveBuffer[0] << 8) | receiveBuffer[1];
    uint16_t rawB = ((uint16_t)receiveBuffer[2] << 8) | receiveBuffer[3];

    // Output for comparison with Nano/SkywalkerSpy
    Serial.printf("%.2f,%u,%u\n", tempC, rawA, rawB);
  } else {
    readAttempts++;
    if (readAttempts > MAX_ATTEMPTS) readAttempts = MAX_ATTEMPTS;
#ifdef __WARN__
    Serial.printf("[!] Failed read (attempt %d)\n", readAttempts);
#endif
  }

  vRingbufferReturnItem(s_rb, (void*)items);
}
