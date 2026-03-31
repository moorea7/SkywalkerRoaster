# ESP32 Migration Plan — WebSocket over WiFi

## Goal

Replace the Arduino microcontroller with an ESP32, enabling Artisan Scope to connect wirelessly over WebSocket instead of USB, while preserving all existing roaster control and monitoring functionality.

---

## Why WebSocket (not raw TCP)

Artisan does not support raw TCP serial connections. Its native network protocols are **MODBUS TCP**, **WebSocket**, and **Phidgets Network Server**. WebSocket is the best fit here:

- Artisan has a dedicated WebSocket device type with configurable command/response mappings
- The ESP32 Arduino ecosystem has a mature WebSocket server library (`arduinoWebSockets`)
- No intermediate bridge process needed on the PC
- Commands and responses are text-based — maps naturally to the existing TC4-style interface

---

## Target Architecture

```
Artisan (PC)
    │
    │  WebSocket (ws://esp32-ip:81)
    ▼
ESP32
  ├── Core 0: WiFi stack + WebSocket server + Artisan command handling
  └── Core 1: Roaster protocol loop (RMT TX/RX, temp calc, failsafe)
        │
        │  GPIO pins (with 3.3V → 5V level shifter)
        ▼
    Roaster hardware (Skywalker/ITOP MTCR)
```

The dual-core split is essential: Core 1 runs the timing-sensitive roaster bit-bang work in isolation from WiFi and WebSocket ISR activity on Core 0.

---

## Hardware Changes

### New hardware required
- **ESP32 development board** (ESP32-WROOM-32 or equivalent)
- **3.3V ↔ 5V level shifter** (bidirectional, e.g. TXS0102 or BSS138-based)
  - Required because ESP32 GPIO is 3.3V; the roaster interface operates at 5V

### Wiring changes
| Signal | Arduino pin | ESP32 GPIO | Notes |
|--------|-------------|------------|-------|
| Roaster RX (read) | Digital 2 | GPIO 4 (or any input-capable) | Via level shifter |
| Roaster TX (write) | Digital 3 | GPIO 5 (or any output-capable) | Via level shifter |
| Power | VIN (5V) | VIN or separate 5V rail | ESP32 has onboard 3.3V reg |
| Ground | GND | GND | Common ground with level shifter |

GPIO numbers are suggestions — any available GPIO works with RMT.

---

## Why RMT is Required

The existing code uses `pulseIn()` and `delayMicroseconds()` for the roaster's custom bit-bang protocol. These are unreliable on ESP32 when WiFi is active because WiFi ISRs introduce latency spikes of 100–500 µs — enough to corrupt the 850 µs difference between a binary `1` (1,500 µs) and binary `0` (650 µs).

ESP32's **RMT (Remote Control Transceiver) peripheral** generates and captures pulse sequences entirely in hardware, independent of the CPU and WiFi scheduler. It is the correct replacement for both `delayMicroseconds()` (TX) and `pulseIn()` (RX).

### Roaster protocol timing reference
| Signal | Duration |
|--------|----------|
| Preamble | 7,500 µs |
| Binary `1` pulse | 1,500 µs |
| Binary `0` pulse | 650 µs |
| Inter-bit gap | 750 µs |
| Full message cycle | ~10 ms |

These constants are preserved unchanged — only the mechanism for generating/measuring them changes.

---

## Artisan WebSocket Device Configuration

Artisan's WebSocket device type (Config → Device → WebSocket) requires:

- **URL:** `ws://<esp32-ip>:81`
- **Command to request data:** e.g. `READ`
- **Response format:** Artisan parses the response string for ET and BT values

The ESP32 WebSocket server receives text frames from Artisan, processes the command, and sends back a text frame response — the same request/response pattern as the existing TC4 serial protocol, just transported over WebSocket instead of USB.

Artisan commands to support (same as current TC4 set):

| Command | WebSocket message | Response |
|---------|------------------|----------|
| `READ` | `READ` | `123.4,456.7` (ET,BT) |
| `OT1,<val>` | `OT1,75` | `#OK` |
| `OT2,<val>` | `OT2,50` | `#OK` |
| `DRUM,<val>` | `DRUM,100` | `#OK` |
| `FILTER,<val>` | `FILTER,1` | `#OK` |
| `COOL,<val>` | `COOL,100` | `#OK` |
| `OFF` | `OFF` | `#OK` |
| `ESTOP` | `ESTOP` | `#OK` |

---

## Migration Phases

### Phase 1 — WebSocket server (Artisan connection)

**Scope:** Establish the WebSocket link between Artisan and the ESP32. No real roaster protocol yet — use stub temperature values.

**Tasks:**
1. Create new sketch `SkyCommandESP32/SkyCommandESP32.ino`
2. Connect ESP32 to WiFi (hardcoded SSID/password initially)
3. Start a `WebSocketsServer` on port 81 using the `arduinoWebSockets` library
4. Implement a `webSocketEvent()` callback that parses incoming text frames and dispatches commands
5. On `READ`, respond with stub values (e.g. `"20.0,20.0"`)
6. On control commands (`OT1`, `OT2`, etc.), store the value and respond `"#OK"`
7. Configure Artisan's WebSocket device to point at the ESP32 IP and verify the connection

**Library:** `arduinoWebSockets` by Markus Sattler — available via Arduino Library Manager as `WebSockets`.

**Artisan configuration:**  
Config → Device → set device to **WebSocket** → enter `ws://<esp32-ip>:81` → map ET/BT to the two values in the `READ` response.

**Success criteria:** Artisan connects over WiFi, polls `READ` on schedule, and receives stub temperature values without errors.

---

### Phase 2 — RMT TX (send commands to roaster)

**Scope:** Replace `pulsePin()` / `sendRoasterMessage()` with RMT-based pulse generation.

**Tasks:**
1. Configure an RMT TX channel on the roaster write GPIO
2. Build a helper `buildRmtFrame(uint8_t* bytes, size_t len)` that converts the 6-byte roaster message into an `rmt_item32_t[]` pulse sequence using the known timing constants
3. Replace the `sendRoasterMessage()` function body with an RMT transmit call
4. Validate output with a logic analyzer against the known protocol timing

**Key implementation note:**  
Each byte is sent LSB-first. Each bit becomes an `rmt_item32_t` with `duration0` set to the pulse length (1500 or 650 µs) and `duration1` set to the inter-bit gap (750 µs). The preamble is a single long item (7500 µs low, 3800 µs high).

**Success criteria:** Logic analyzer confirms correct pulse widths and byte values for a known command.

---

### Phase 3 — RMT RX (receive temperature from roaster)

**Scope:** Replace `receiveSerialBitsFromRoaster()` / `pulseIn()` with RMT-based pulse capture.

This is the most complex phase.

**Tasks:**
1. Configure an RMT RX channel on the roaster read GPIO
2. Set the RMT filter threshold to reject glitches < 400 µs
3. Set the RMT idle threshold to ~9000 µs (end-of-message marker)
4. In the receive loop, read the captured `rmt_item32_t[]` array
5. Decode each item: `duration0 > 1200 µs` → bit `1`, else bit `0` (same threshold as current code)
6. Reconstruct bytes LSB-first
7. Detect the preamble item (`duration0 > 5000 µs`) to align frame boundaries
8. Validate checksum; on failure increment `roasterReadAttempts` as before
9. Feed decoded ADC values A and B into the existing temperature polynomial calculation
10. Replace the stub `READ` response with real temperature values

**Success criteria:** Live temperature readings from the roaster displayed correctly in Artisan over WebSocket.

---

### Phase 4 — Dual-core task split

**Scope:** Move the roaster protocol loop to Core 1 to fully isolate it from WiFi and WebSocket activity.

**Tasks:**
1. Extract the roaster read/write loop into a FreeRTOS task function `roasterTask(void* params)`
2. Pin it to Core 1: `xTaskCreatePinnedToCore(roasterTask, "roaster", 4096, NULL, 2, NULL, 1)`
3. Share state between cores using a mutex-protected struct (current temps, pending commands)
4. Keep WiFi, WebSocket server, and Artisan command handling on Core 0 (the default Arduino `loop()`)
5. Validate that the failsafe watchdog (10-second timeout) still functions correctly across cores

**Success criteria:** No timing degradation in roaster communication during active WebSocket data transfer.

---

### Phase 5 — WiFi provisioning

**Scope:** Replace hardcoded SSID/password with a runtime configuration mechanism.

**Options (choose one):**
- **WiFiManager library** — on first boot, ESP32 becomes an AP with a captive portal for entering credentials; stores to flash
- **Hardcoded with OTA** — simplest option; update credentials via Arduino OTA when needed
- **BLE provisioning** — use ESP32 BLE to receive credentials from a phone app (most complex)

**Recommended:** WiFiManager for ease of use without requiring reflashing.

**Tasks:**
1. Add WiFiManager (or chosen method) to the sketch
2. Store credentials in NVS (non-volatile storage) via `Preferences` library
3. Add a hardware reset button (one GPIO → GND) to clear credentials and re-enter provisioning mode

**Success criteria:** Credentials survive power cycles; provisioning works without USB cable.

---

### Phase 6 — Spy sketch port

**Scope:** Port `SkywalkerSpy` to ESP32 for passive logging over WebSocket.

**Tasks:**
1. Create `SkywalkerSpyESP32/SkywalkerSpyESP32.ino`
2. Apply the same RMT RX approach from Phase 3 to both controller and roaster Tx lines
3. Replace blocking `pulseIn()` (currently has no timeout — will deadlock on ESP32) with RMT capture
4. Stream `TEMP,HEAT_DUTY,VENT_DUTY` responses over WebSocket instead of USB serial

**Success criteria:** Artisan receives passive temperature + duty cycle log over WebSocket.

---

### Phase 7 — CI/CD and housekeeping

**Tasks:**
1. Add ESP32 board to the GitHub Actions compile matrix:
   ```yaml
   - esp32:esp32:esp32
   ```
2. Add `esp32:esp32` platform to `compile-sketch.yml` alongside `arduino:avr`
3. Update `CLAUDE.md` with ESP32 sketch locations, RMT conventions, and WebSocket library dependency
4. Update `README.md` with new wiring diagram, Artisan WebSocket config instructions, and provisioning steps
5. Archive (do not delete) the original AVR sketches — keep them working for users without ESP32 hardware

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| RMT RX frame alignment issues (missed preamble) | Medium | High | Add idle-gap detection; test with logic analyzer before Artisan integration |
| WebSocket reconnect causes Artisan dropout | Low | Medium | ESP32 WebSocket library handles reconnects; Artisan timeout is configurable |
| Roaster failsafe triggers during WiFi reconnect | Medium | Medium | Cache last valid command; retransmit on reconnect; Core 1 continues independently |
| Level shifter introduces signal delay | Low | Medium | Use fast level shifter (TXS0102); validate with logic analyzer |
| RMT timing drift at temperature | Low | Low | Timing constants have wide margins; monitor in field |
| Artisan WebSocket message framing mismatch | Low | Medium | Test command/response format against Artisan WebSocket device docs before Phase 3 |

---

## What Does Not Change

The following are direct ports with no logic changes required:

- Roaster protocol timing constants (preamble, bit durations, inter-bit gap)
- Temperature polynomial coefficients (16 coefficients, 4th-degree)
- Command set and semantics (READ, OT1, OT2, DRUM, FILTER, COOL, OFF, ESTOP)
- Checksum validation logic
- Failsafe timeout (10 seconds)
- Maximum temperature limit (300°C)
- `Data/model3.py` and all temperature training data

---

## Dependencies

| Library | Purpose | Source |
|---------|---------|--------|
| `WiFi.h` | WiFi connection | Arduino-ESP32 built-in |
| `WebSocketsServer.h` | WebSocket server | [arduinoWebSockets](https://github.com/Links2004/arduinoWebSockets) via Library Manager |
| `driver/rmt.h` | RMT peripheral (TX + RX) | ESP-IDF / Arduino-ESP32 built-in |
| `Preferences.h` | NVS credential storage | Arduino-ESP32 built-in |
| WiFiManager (optional) | Captive portal provisioning | [tzapu/WiFiManager](https://github.com/tzapu/WiFiManager) |

---

## Recommended Development Order

```
Phase 1 (WebSocket + Artisan)  →  verify Artisan WiFi connection works
Phase 2 (RMT TX)               →  verify roaster receives correct commands
Phase 3 (RMT RX)               →  verify temperature reads correctly
Phase 4 (dual-core)            →  stress test timing under WiFi load
Phase 5 (provisioning)         →  quality-of-life, do last
Phase 6 (Spy port)             →  independent, can be done any time after Phase 3
Phase 7 (CI/CD)                →  done alongside each phase
```

Each phase produces a testable, standalone result. Do not proceed to the next phase until the current one is validated against the real roaster hardware.
