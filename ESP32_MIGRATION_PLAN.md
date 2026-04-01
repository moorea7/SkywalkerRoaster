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

The current solution (ELEGOO Nano, ATmega328P) operates at **5V logic**. The ESP32 operates at **3.3V logic**. Each signal line is unidirectional, so each requires a different level-shifting technique.

### Bill of Materials

#### Required

| Qty | Component | Purpose | Notes |
|-----|-----------|---------|-------|
| 1 | ESP32-WROOM-32 dev board | Microcontroller | ESP32 DevKitC or equivalent |
| 1 | 74HCT1G125 single buffer (SOT-23-5) | 3.3V → 5V on TX line | HCT family: accepts 3.3V in, outputs 5V; non-inverting; ~$0.30 |
| 1 | 1 kΩ resistor (1/4W) | Voltage divider on RX line | |
| 1 | 2 kΩ resistor (1/4W) | Voltage divider on RX line | |
| 1 | 100 nF ceramic capacitor | Bypass cap for 74HCT VCC | Place close to IC pins |
| — | Hookup wire / breadboard | Connections | |

#### Optional — ET probe (exhaust temperature, K-type thermocouple)

Provides an independent ET channel in Artisan and a redundant over-temperature safety path that does not rely on the roaster's internal ADC or the polynomial model.

| Qty | Component | Purpose | Notes |
|-----|-----------|---------|-------|
| 1 | MAX31855 breakout board | K-type thermocouple amplifier + cold junction compensation | Adafruit #269 or equivalent; 3.3V native — no level shifter needed |
| 1 | K-type thermocouple probe | Exhaust / ET temperature sensing | Choose probe length/type to suit roaster exhaust port |

#### Optional — ambient probe

Provides room/ambient temperature as an additional Artisan channel. Useful for environment logging and charge temperature reference.

| Qty | Component | Purpose | Notes |
|-----|-----------|---------|-------|
| 1 | DS18B20 (TO-92 or waterproof probe) | 1-Wire digital temperature sensor | 3.3V compatible; ±0.5°C accuracy |
| 1 | 4.7 kΩ resistor (1/4W) | 1-Wire pull-up to 3.3V | Required by 1-Wire protocol |

**Why these components:**
- **74HCT1G125** — HCT-family logic accepts 3.3V as a valid HIGH (V_IH min = 2.0V) and outputs at VCC (5V). Non-inverting with active-low output enable tied to GND. Cleanest solution for boosting the TX line.
- **Resistor divider (1kΩ + 2kΩ)** — Passively divides 5V to 3.33V on the RX line. No active components needed for a signal going *into* the ESP32.
- **MAX31855** — SPI interface, built-in cold junction compensation, 14-bit resolution, 3.3V native. No level shifter required.
- **DS18B20** — 1-Wire protocol, single GPIO + pull-up resistor. Multiple sensors can share one pin if needed.

### Wiring diagram

```
ROASTER USB CABLE
─────────────────────────────────────────────────────────────

  Red  ──────────────────────── 5V rail ──┬── 74HCT1G125 VCC (pin 5)
                                          └── 100nF cap ── GND

  Black ─────────────────────── GND ─────┬── ESP32 GND
                                         └── 74HCT1G125 GND (pin 2)

  Green (Roaster → ESP32 RX)
    ├── R1 (1kΩ) ──┬── R2 (2kΩ) ── GND
                   └──────────────────── ESP32 GPIO 4 (RX, 3.3V input)

    [5V signal divided to 3.33V before reaching ESP32]

  White (ESP32 TX → Roaster)
    └── 74HCT1G125 output (pin 4) ──────── White wire onward to roaster

         74HCT1G125 (SOT-23-5)
         ┌─────────┐
         │ /OE (2) ├── GND  (always enabled)
         │  A  (3) ├── ESP32 GPIO 5 (TX, 3.3V output)
         │  Y  (4) ├── White wire → roaster (5V)
         │ VCC (5) ├── 5V rail
         │ GND (1) ├── GND
         └─────────┘


OPTIONAL — ET probe (MAX31855 + K-type thermocouple)
─────────────────────────────────────────────────────────────

  ESP32 3.3V ──── MAX31855 VCC
  ESP32 GND  ──── MAX31855 GND
  ESP32 GPIO 18 ── MAX31855 CLK   (SPI clock, shared if adding more SPI devices)
  ESP32 GPIO 19 ── MAX31855 DO    (MISO — read only, no MOSI connection needed)
  ESP32 GPIO 15 ── MAX31855 CS    (chip select)

  MAX31855 T+ / T- ──── K-type thermocouple leads (polarity marked on board)

  [All signals 3.3V — no level shifter required]


OPTIONAL — Ambient probe (DS18B20)
─────────────────────────────────────────────────────────────

  ESP32 3.3V ──┬── 4.7kΩ pull-up ──┬── ESP32 GPIO 17 (1-Wire data)
               │                   └── DS18B20 DATA pin
               └── DS18B20 VCC

  ESP32 GND  ──── DS18B20 GND

  [3.3V native — no level shifter required]
```

### Wiring table

| Signal | ESP32 GPIO | Interface | Notes |
|--------|------------|-----------|-------|
| Roaster RX / read (Green wire) | GPIO 4 | RMT RX | 5V → 3.3V via 1kΩ + 2kΩ divider |
| Roaster TX / write (White wire) | GPIO 5 | RMT TX | 3.3V → 5V via 74HCT1G125 |
| Power (Red wire) | VIN | — | Direct; also powers 74HCT1G125 |
| Ground (Black wire) | GND | — | Common ground |
| MAX31855 CLK *(optional)* | GPIO 18 | SPI | ET probe clock |
| MAX31855 DO *(optional)* | GPIO 19 | SPI MISO | ET probe data |
| MAX31855 CS *(optional)* | GPIO 15 | SPI CS | ET probe chip select |
| DS18B20 DATA *(optional)* | GPIO 17 | 1-Wire | Ambient probe; 4.7kΩ pull-up to 3.3V |

GPIO numbers are suggestions — any available GPIO works for each function.

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

## Safety Principle

**Never transmit to the roaster without a verified, working temperature read-back.**

The over-temperature cutoff (300°C) and all safety logic depend entirely on the RX path being correct. A broken or unvalidated RX path silently disables the primary protection while the roaster is live. Equally, a TX bug could send a garbage heat command with no way to detect it. For these reasons:

- All read logic is built and validated before any write logic
- Write capability is introduced one output at a time, in order of increasing hazard
- The heat/burner output is enabled last, only after every other output and all safety paths are confirmed working

---

## Migration Phases

### Phase 1 — RMT RX: passive read (Spy mode)

**Scope:** Port `SkywalkerSpy` to ESP32 using RMT. Receive-only — no transmission to the roaster at any point in this phase.

This is the safest possible starting point. The ESP32 only listens; the existing Nano can remain connected in parallel as a ground truth reference.

**Tasks:**
1. Create `SkywalkerSpyESP32/SkywalkerSpyESP32.ino`
2. Configure RMT RX channel on the roaster read GPIO
3. Set RMT filter threshold to reject glitches < 400 µs
4. Set RMT idle threshold to ~9000 µs (end-of-message marker)
5. Decode captured `rmt_item32_t[]` array: `duration0 > 1200 µs` → bit `1`, else bit `0`
6. Reconstruct bytes LSB-first; detect preamble item (`duration0 > 5000 µs`) for frame alignment
7. Validate checksum on every frame
8. Apply existing temperature polynomial to decoded ADC values A and B
9. Output `TEMP,HEAT_DUTY,VENT_DUTY` over USB serial

**Validation:** Run ESP32 and existing Nano side-by-side. Compare temperature readings across a full heat cycle. Values must agree within 1°C before proceeding.

**Success criteria:** ESP32 temperature readings match the known-good Nano over USB serial, validated across a real heat cycle.

---

### Phase 2 — WebSocket server (Artisan connection, read-only)

**Scope:** Add WiFi and WebSocket transport. Artisan connects and receives live temperature — still no TX to the roaster.

**Tasks:**
1. Create `SkyCommandESP32/SkyCommandESP32.ino` based on the Spy sketch
2. Connect ESP32 to WiFi (hardcoded SSID/password initially)
3. Start a `WebSocketsServer` on port 81 using the `arduinoWebSockets` library
4. Implement `webSocketEvent()` callback: respond to `READ` with real temperature values from Phase 1
5. Reject all control commands (`OT1`, `OT2`, `DRUM`, `COOL`, `OFF`, `ESTOP`) with `#READONLY` — do not act on them
6. Configure Artisan WebSocket device: Config → Device → WebSocket → `ws://<esp32-ip>:81`

**Library:** `arduinoWebSockets` by Markus Sattler — available via Arduino Library Manager as `WebSockets`.

**Success criteria:** Artisan displays live roaster temperature over WiFi. Control commands are received but explicitly rejected.

---

### Phase 3 — Dual-core task split

**Scope:** Move the roaster RX loop to Core 1 before introducing any TX. This isolates timing-sensitive RX from WiFi ISR activity and is the correct foundation for all subsequent write phases.

**Tasks:**
1. Extract the roaster read loop into a FreeRTOS task `roasterTask(void* params)`
2. Pin to Core 1: `xTaskCreatePinnedToCore(roasterTask, "roaster", 4096, NULL, 2, NULL, 1)`
3. Share temperature state to Core 0 via mutex-protected struct
4. Keep WiFi, WebSocket server, and Artisan handling on Core 0
5. Validate temperature readings remain stable under active WebSocket traffic
6. Validate the 10-second failsafe watchdog still triggers correctly across cores

**Success criteria:** No temperature read degradation during sustained WebSocket traffic. Failsafe confirmed working.

---

### Phase 4 — RMT TX: shutdown and emergency stop

**Scope:** Introduce the first write path — commands that put the roaster into a safe state. These are the lowest-risk outputs: they reduce heat and stop operation.

**Tasks:**
1. Configure RMT TX channel on the roaster write GPIO
2. Implement `buildRmtFrame(uint8_t* bytes, size_t len)` — converts 6-byte message to `rmt_item32_t[]` pulse sequence (LSB-first; preamble 7500 µs low / 3800 µs high; bit `1` = 1500 µs, bit `0` = 650 µs, inter-bit gap 750 µs)
3. Implement `OFF` command: transmit all-zero duty cycles (heat=0, fan=0, drum=0)
4. Implement `ESTOP` command: same as OFF but set internal eStop flag to block further commands
5. Validate both commands on a logic analyzer before connecting to roaster
6. Connect to roaster and verify `OFF` produces correct shutdown behaviour
7. Remove `#READONLY` rejection for `OFF` and `ESTOP` in the WebSocket handler

**Success criteria:** `OFF` and `ESTOP` commands transmitted correctly; roaster shuts down cleanly on receipt.

---

### Phase 5 — RMT TX: bean cooler

**Scope:** Enable the `COOL` command (cooling fan output). This is safe to enable early — activating the cooler cannot raise temperature.

**Tasks:**
1. Implement `COOL` command: set cool fan byte in the 6-byte message; all other outputs remain at last safe values
2. Validate on logic analyzer
3. Test on roaster: confirm cooling fan activates and temperature is tracked correctly throughout
4. Remove `#READONLY` rejection for `COOL`

**Success criteria:** Cooling fan activates on `COOL` command; temperature continues to read correctly.

---

### Phase 6 — RMT TX: drum

**Scope:** Enable the `DRUM` command (drum motor output).

**Tasks:**
1. Implement `DRUM` command: set drum byte in the 6-byte message
2. Validate on logic analyzer
3. Test on roaster: confirm drum starts/stops correctly; confirm no interference with temperature readings
4. Remove `#READONLY` rejection for `DRUM`

**Success criteria:** Drum motor responds correctly to `DRUM` commands at all specified speeds.

---

### Phase 7 — RMT TX: air / vent

**Scope:** Enable the `OT2` command (vent/air fan output). Air affects roast dynamics but cannot directly cause overheating.

**Tasks:**
1. Implement `OT2` command: set vent duty byte in the 6-byte message (0–100)
2. Validate on logic analyzer
3. Test on roaster: confirm air fan responds proportionally; verify temperature readings remain stable across fan speeds
4. Remove `#READONLY` rejection for `OT2`

**Success criteria:** Vent fan responds correctly across the full 0–100 duty cycle range.

---

### Phase 8 — RMT TX: heat / burner

**Scope:** Enable the `OT1` command (heat/burner output). This is the highest-risk output and is enabled last, only after all read paths, safety logic, and every other output have been individually validated.

**Pre-conditions — all must be met before starting this phase:**
- Temperature readings verified accurate across full roast range (Phase 1–3)
- Over-temperature cutoff (300°C) confirmed functional via a controlled test
- Failsafe timeout confirmed functional (disconnect Artisan; roaster must shut down within 10 seconds)
- `OFF` and `ESTOP` confirmed working (Phase 4)
- Cooling fan confirmed working (Phase 5)

**Tasks:**
1. Implement `OT1` command: set heat duty byte in the 6-byte message (0–100)
2. Validate on logic analyzer at several duty values (0, 25, 50, 75, 100)
3. Initial live test: start at low duty (e.g. 20%), confirm temperature rises proportionally, confirm `OFF` stops heating immediately
4. Test over-temperature cutoff: raise heat until 300°C limit; confirm emergency stop fires
5. Test 10-second failsafe under heat: kill WebSocket connection mid-roast; confirm roaster shuts down
6. Remove `#READONLY` rejection for `OT1`

**Success criteria:** Heat output responds correctly; over-temperature cutoff and failsafe both confirmed under real heat load.

---

### Phase 9 — WiFi provisioning

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

### Phase 10 — CI/CD and housekeeping

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
| RMT RX frame alignment issues (missed preamble) | Medium | High | Add idle-gap detection; validate against Nano readings before any TX phase |
| RX path silently fails during a roast | Low | High | Watchdog on RX: if no valid frame received in 2 seconds, trigger ESTOP |
| RMT TX sends incorrect heat duty | Low | High | Logic analyzer validation required before connecting to roaster |
| Over-temp cutoff non-functional if RX is broken | Medium | High | Explicitly test cutoff under real heat load in Phase 8 pre-conditions |
| WebSocket reconnect causes Artisan dropout | Low | Medium | ESP32 WebSocket library handles reconnects; Artisan timeout is configurable |
| Roaster failsafe triggers during WiFi reconnect | Medium | Medium | Cache last valid command; retransmit on reconnect; Core 1 continues independently |
| Level shifter introduces signal delay | Low | Medium | Validate with logic analyzer in Phase 1 before any live roaster connection |
| RMT timing drift at temperature | Low | Low | Timing constants have wide margins; monitor in field |
| Artisan WebSocket message framing mismatch | Low | Medium | Test command/response format in Phase 2 before any TX is enabled |

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
| `Adafruit_MAX31855.h` *(optional)* | ET probe — K-type thermocouple via MAX31855 | [Adafruit MAX31855](https://github.com/adafruit/Adafruit-MAX31855-library) via Library Manager |
| `DallasTemperature.h` + `OneWire.h` *(optional)* | Ambient probe — DS18B20 | [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library) + [OneWire](https://github.com/PaulStoffregen/OneWire) via Library Manager |
| WiFiManager *(optional)* | Captive portal provisioning | [tzapu/WiFiManager](https://github.com/tzapu/WiFiManager) |

---

## Development Order Summary

```
Phase 1  (RMT RX, Spy mode)       →  validate reads against Nano before anything else
Phase 2  (WebSocket, read-only)    →  Artisan sees live temp; all control commands rejected
Phase 3  (Dual-core split)         →  isolate RX from WiFi; confirm failsafe across cores
Phase 4  (TX: shutdown / ESTOP)    →  first writes; safe-state commands only
Phase 5  (TX: bean cooler)         →  cooling fan; cannot raise temperature
Phase 6  (TX: drum)                →  drum motor; no thermal risk
Phase 7  (TX: air / vent)          →  airflow control; no direct heat risk
Phase 8  (TX: heat / burner)       →  last; all safety paths confirmed before enabling
Phase 9  (WiFi provisioning)       →  quality-of-life; do last
Phase 10 (CI/CD)                   →  done incrementally alongside each phase
```

**Do not proceed to the next phase until the current one is validated against real roaster hardware.**
