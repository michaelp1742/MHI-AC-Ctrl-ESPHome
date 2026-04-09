# MHI-AC-Ctrl ESP8266 NMI-Hardened SPI Fix — Technical Brief

## Purpose

This document describes the NMI-hardened SPI fix applied to the
MHI-AC-Ctrl ESPHome component. It contains the investigation findings,
the implementation details, and test results.

The fork is https://github.com/michaelp1742/MHI-AC-Ctrl-ESPHome
(branch `nmi-hardened`), forked from
https://github.com/ginkage/MHI-AC-Ctrl-ESPHome (v4.x, v4.3.1).
The modified files are `components/MhiAcCtrl/MHI-AC-Ctrl-core.cpp`
and `components/MhiAcCtrl/MHI-AC-Ctrl-core.h`.

---

## CRITICAL: This Is NOT an ESP8266-Only Problem

The ginkage codebase uses the SAME software SPI bit-bang implementation
on ALL platforms — ESP8266, ESP32, ESP32-S3, ESP32-C3. The
`MHI-AC-Ctrl-core.cpp` file is shared across all of them.

### Evidence: ESP32-S3 UniversalAircoController Has Identical Errors

Issue #196 on ginkage's repo (Dec 2025) reports a user with the
TinyTronics "Universal Air Conditioning Controller" — an ESP32-S3
board — getting the same -2 and -1 errors with frame_size 33:

    https://github.com/ginkage/MHI-AC-Ctrl-ESPHome/issues/196

The user's YAML shows: `esp32: board: esp32-s3-devkitc-1` with pins
`sck_pin: 21`, `miso_pin: 34`, `mosi_pin: 48`. The errors are
identical to what we see on ESP8266 — checksum failures and signature
mismatches caused by interrupt preemption during the bit-bang loop.

Issue #121 reports the same on ESP32 Devkit1 — errors -1 with the
AC powering off after 60-120 seconds (the AC interprets corrupted
MISO frames as invalid commands).

### Why ESP32 Has the Same Problem (Differently)

The ESP32 family has a different interrupt architecture (FreeRTOS with
multiple priority levels, dual core on some variants), but the
fundamental issue is the same: the software bit-bang loop in
`MHI-AC-Ctrl-core.cpp` uses `digitalRead()`/`digitalWrite()` with
busy-wait polling, and ANY interrupt preemption during the ~10-16ms
frame exchange corrupts the data.

On ESP32, the WiFi stack runs on a separate task at high priority
and can preempt user code. On ESP32-S3, the additional USB-OTG and
other peripheral interrupts add further preemption sources.

### The Correct Solution for ESP32: hberntsen's Hardware SPI

For ESP32/ESP32-S3/ESP32-C3, the proper fix is to use hberntsen's
`mhi-ac-ctrl-esp32` implementation which uses the hardware SPI slave
peripheral + RMT for CS synthesis:

    https://github.com/hberntsen/mhi-ac-ctrl-esp32

This is a completely different codebase from ginkage — different
component name, different YAML config, different architecture. It
eliminates the bit-bang entirely. The ESP32-S3 and ESP32-C3 are
explicitly supported. The ESP32 plain also works (see issue #11
for pin selection).

### The TinyTronics UniversalAircoController Board

This is a commercial ESP32-S3 board sold at tinytronics.nl, designed
by fonske (a ginkage contributor). It has:
- ESP32-S3 SoC
- Bi-directional 3.3V ↔ 5V level shifter
- Step-down buck converter for higher input voltages
- JST-GH connector + solderable USB-A
- 3 GPIO pins exposed (SCK/MOSI/MISO)
- USB-C for programming

The ginkage repo has a `UniversalAircoController/` directory with
YAML examples for this board. BUT: it still uses the same software
SPI bit-bang code, which is why issue #196 exists.

Users of this board should ideally switch to hberntsen's component.
The pins in their YAML (21, 34, 48) would need to be mapped to
hberntsen's config format.

### Scope of Our Fix

Our NMI-blocking fix targets the ESP8266 specifically, because:
1. ESP8266 has no alternative (can't run hberntsen's HW SPI)
2. The `xt_rsil(15)` mechanism is ESP8266/LX106-specific
3. The GPIO register addresses are ESP8266-specific
4. The CCOUNT cycle counter access is Xtensa-specific

The `#ifdef ESP8266` / `#else` structure in our patch preserves the
original code for ESP32 platforms. ESP32 users with errors should be
directed to hberntsen's implementation instead.

However, a NOTE or comment in the ESP32 `#else` path would be
helpful to guide people to the right solution.

---

## The MHI SPI Protocol

The Mitsubishi Heavy Industries air conditioner communicates via a
non-standard SPI-like protocol over three wires: SCK, MOSI, MISO.
There is NO chip select (CS/SS) signal.

### Protocol Parameters

- **Master**: The AC unit (always master, never relinquishes)
- **Slave**: The ESP8266 (or MHI's own WiFi module WF-RAC)
- **Clock polarity**: CPOL=1 (SCK idles HIGH)
- **Clock phase**: CPHA=1 (data captured on rising edge, changed on falling edge)
- **Bit order**: LSB first
- **Bit period**: ~250µs (125µs per half-cycle)
- **Frame size**: 20 bytes (legacy) or 33 bytes (WF-RAC units with vanes LR / 3D auto)
- **Frame duration**: ~10ms for 20 bytes, ~16ms for 33 bytes
- **Inter-frame pause**: ~40ms (SCK stays HIGH/idle)
- **Frame rate**: ~20 frames per second
- **Data exchange**: Full duplex — MOSI (AC→ESP) and MISO (ESP→AC) transfer simultaneously

### Frame Structure

Each frame has: 3 signature bytes + 15 data bytes + 2 checksum bytes = 20 bytes.
For 33-byte frames: additional 13 data bytes + 1 checksum byte.

- MOSI signature: `0x6c` (or `0x6d` for some models), `0x80`, `0x04`
- MISO signature: `0xA9`, `0x00`, `0x07` (20-byte) or `0xAA`, `0x00`, `0x07` (33-byte)
- Checksum: sum of all bytes before the checksum position

### Frame Start Detection

The current code detects frame start by waiting for SCK to remain HIGH
for 5ms continuously. Since the inter-frame gap is ~40ms of idle-high SCK,
this reliably catches the gap. Any SCK low pulse during the wait resets
the 5ms counter.

### Command Protocol (ESP→AC via MISO)

Commands are sent by setting specific bits in the MISO data bytes:

- DB0: Power (bit 1 = set flag, bit 0 = on/off), Mode (bit 5 = set flag, bits 4:2 = mode), Vanes swing (bits 7:6)
- DB1: Fan (bit 3 = set flag, bits 2:0 = speed), Vanes position (bit 7 = set flag, bits 6:4 = position)
- DB2: Temperature setpoint (bit 7 = set flag, bits 6:0 = temp * 2)
- DB3: Room temperature (always sent, from internal sensor or external via MQTT/DS18x20)
- DB6/DB9: Operating data requests
- DB14: Doubleframe toggle (bit 2 toggles each frame)
- DB16/DB17: VanesLR and 3D auto (33-byte frames only)

Commands use a one-shot pattern: the `new_*` variables (e.g. `new_Power`,
`new_Mode`) are loaded into `MISO_frame[]` then immediately zeroed. The
command is consumed regardless of whether the frame exchange succeeds.
This is the existing design — we do not change it.

The MISO frame is fully assembled (including checksums) BEFORE the
bit-bang exchange loop starts. The exchange loop only reads from the
pre-built `MISO_frame[]` array. Both the MISO assembly and the
bit-bang loop run under NMI block on ESP8266. The assembly is pure
computation and safe under interrupt blocking.

---

## The ESP8266 Hardware Architecture (Relevant Parts)

### CPU

- Tensilica Xtensa LX106, single core
- 80MHz or 160MHz (we run at 160MHz via `board_build.f_cpu: 160000000L`)
- 64KB instruction RAM, 96KB data RAM
- CALL0 ABI, 16-entry register file

### Interrupt Architecture

The LX106 has only TWO interrupt priority levels (despite Xtensa
supporting up to 15):

- **Level 1**: All normal maskable interrupts (SPI, UART, GPIO, timers, software)
- **Level 3 (NMI)**: Non-Maskable Interrupt — used by WiFi `wDev` driver and optionally by `hw_timer` in NMI mode

The current interrupt level is stored in CINTLEVEL (4-bit field in the
PS register). Only interrupts at levels ABOVE CINTLEVEL fire.

**Key functions:**
- `ets_intr_lock()` / `noInterrupts()`: Execute `rsil a2, 3` — sets CINTLEVEL=3, blocking level 1 only. **WiFi NMI still fires.**
- `xt_rsil(15)`: Sets CINTLEVEL=15 — blocks EVERYTHING including NMI. Returns previous PS value for restoration.
- `xt_wsr_ps(saved)`: Restores PS register (re-enables interrupts to previous state).

The `xt_rsil`/`xt_wsr_ps` macros are defined in the ESP8266 Arduino core headers.
They compile to single `rsil`/`wsr` Xtensa instructions. If the macros aren't
available (very old core), they can be defined as:

```c
#define xt_rsil(level) (__extension__({uint32_t state; \
  __asm__ __volatile__("rsil %0," __STRINGIFY(level) : "=a" (state)); state;}))
#define xt_wsr_ps(state) __asm__ __volatile__("wsr %0,ps; isync" :: "a" (state))
```

### WiFi NMI — What It Does and Why It Exists

The WiFi radio's `wDev` module fires NMI when:
- A frame has been received into a DMA buffer and needs to be drained to a pbuf
- A frame transmission has completed

The NMI handler is short — it moves data between hardware DMA descriptors
and software `pbuf` queues. Typical execution time: tens of microseconds.

If NMI is blocked, DMA descriptors stay occupied. The hardware has a ring
of 5-7 DMA descriptors. At typical home network traffic rates, 10ms of
blocked NMI accumulates 0-3 frames in the hardware ring — well within
capacity.

802.11 ACK frames (SIFS timing, 10µs after receipt) are generated
entirely in the MAC hardware. They do NOT require CPU/NMI involvement.
The AP will NOT retransmit frames just because NMI is blocked for 10ms.

### GPIO Registers

ESP8266 GPIO is memory-mapped at fixed addresses:

```c
#define MHI_GPIO_IN    (*(volatile uint32_t*)0x60000318)  // Read pin state
#define MHI_GPIO_SET   (*(volatile uint32_t*)0x60000304)  // Set pins HIGH
#define MHI_GPIO_CLR   (*(volatile uint32_t*)0x60000308)  // Set pins LOW
```

Each GPIO pin corresponds to a bit position (e.g., GPIO14 = bit 14).
Direct register access takes ~100ns vs ~3-5µs for `digitalRead()`/`digitalWrite()`.

Pin numbers used by MHI-AC-Ctrl (configurable in YAML, defaults in ginkage v4.x):
- SCK: GPIO14 (D5 on Wemos D1 Mini)
- MOSI: GPIO13 (D7)
- MISO: GPIO12 (D6)

### CPU Cycle Counter (CCOUNT)

The Xtensa LX106 has a 32-bit cycle count register that increments
every CPU clock cycle, regardless of interrupt state. At 160MHz it
wraps every ~26.8 seconds.

```c
#define MHI_GET_CCOUNT() (__extension__({uint32_t c; \
  __asm__ __volatile__("rsr %0,ccount" : "=a" (c)); c;}))
```

This is needed because `millis()` depends on timer interrupts, which
are blocked when we use `xt_rsil(15)`. The cycle counter keeps ticking
regardless.

Timeout calculation: `max_time_ms * 160000UL` gives cycles.
Unsigned subtraction `(MHI_GET_CCOUNT() - start_ccount) > timeout_cycles`
handles 32-bit wrap correctly.

---

## The Current Problem — Root Cause Analysis

### What Happens Now

The bit-bang loop in `MHI_AC_Ctrl_Core::loop()` does this for each bit:

1. `while (digitalRead(SCK_PIN)) {}` — busy-wait for falling edge
2. `digitalWrite(MISO_PIN, bit)` — set output
3. `while (!digitalRead(SCK_PIN)) {}` — busy-wait for rising edge
4. `if (digitalRead(MOSI_PIN))` — sample input

This runs 8 × frameSize times (160 iterations for 20-byte frame,
264 for 33-byte). Total time: ~10-16ms of tight polling.

The WiFi NMI fires asynchronously — whenever the radio receives or
finishes transmitting a frame. This preempts the bit-bang loop at
arbitrary points. When the NMI handler runs (even for just 10-50µs),
the ESP8266 misses one or more SCK edges from the AC. The AC doesn't
wait — it continues clocking. The result is:

- **Missed falling edge**: The code is in `while (digitalRead(SCK_PIN))`
  when the NMI fires. By the time it returns, the falling edge AND the
  rising edge may have already passed. The loop is now desynchronised.
  If it's at the end of the frame, SCK returns to idle-high and the
  timeout fires → **error -4** (SCK stuck high).

- **Bit slip**: If the NMI preemption causes the code to miss exactly
  one full bit cycle, subsequent bits are read one position off. This
  corrupts the frame data → **error -2** (bad checksum).

- **Frame slip**: If the NMI causes loss of synchronisation near the
  frame boundary, the next `loop()` call picks up mid-frame data as
  if it were a frame start → **error -1** (bad signature).

### Error Distribution From User's Logs

288 error lines over ~15 minutes of operation:

| Error | Code | Count | Percentage | Meaning |
|-------|------|-------|------------|---------|
| SCK stuck high | -4 | ~248 | 86% | Missed edge → timeout |
| Invalid signature | -1 | ~28 | 10% | Frame desync |
| Invalid checksum | -2 | ~12 | 4% | Bit corruption |
| SCK stuck low | -3 | 0 | 0% | Would indicate HW fault (none seen) |

The error rate is ~1 error every 3-6 seconds, correlating with WiFi
beacon intervals and background WiFi maintenance. The zero count for
error -3 confirms the hardware wiring is fine.

### Why ESPHome Makes It Worse Than Arduino Sketch

ESPHome runs a heavier event loop: native API server, OTA listener,
sensor polling, WiFi management, mDNS, logging. This generates more
frequent WiFi traffic and longer interrupt service times compared to
a bare Arduino sketch that only runs MQTT. The ginkage README
acknowledges the component regularly exceeds ESPHome's 30ms blocking
limit.

### Why absalom-muc Abandoned Hardware SPI

The ESP8266 HSPI slave peripheral has a protocol mismatch with MHI:

1. **Mandatory command/address phase**: The ESP8266 SPI slave requires
   a minimum 3-bit command before data transfer. The MHI AC sends raw
   data bytes with no command framing. The `slv_cmd_define` register
   allows custom commands but cannot disable the command phase entirely.

2. **Requires CS (chip select)**: The ESP8266 SPI slave resets its
   internal state if CS goes high during a transfer. MHI has no CS signal.

These are hardware limitations that cannot be worked around in software
on the ESP8266's SPI peripheral.

### Why the ESP32 Implementation (hberntsen) Works

The ESP32's SPI slave peripheral is more flexible — it can handle raw
data without command/address phases. The ESP32 implementation also uses
the RMT (Remote Control Transceiver) peripheral to synthesise a CS
signal from the SCK inter-frame gap, solving the CS problem in hardware.
This approach cannot be used on ESP8266 (no RMT peripheral).

---

## The Fix — NMI Blocking

### Approach

Block all interrupts including WiFi NMI using `xt_rsil(15)` from
immediately after frame-start detection through the end of the SPI
bit-bang exchange. This covers both MISO frame assembly and the
bit-bang loop. Also replace `digitalRead()`/`digitalWrite()` with
direct GPIO register access and `millis()` with CCOUNT for timeout.

### Implementation Note: Why NMI Block Covers MISO Assembly

The initial implementation only blocked NMI around the bit-bang loop
itself, leaving MISO frame assembly unprotected. Testing showed this
reduced errors by ~84% but introduced new -3 (SCK stuck low) errors
that weren't present before. Root cause: the ~50-100µs window between
frame-start detection completing and the bit-bang loop starting was
vulnerable to NMI preemption. If an NMI fired during MISO assembly,
the AC had already started clocking by the time the bit-bang began,
causing the code to enter mid-bit-cycle.

Moving `xt_rsil(15)` to immediately after frame-start detection
eliminated this window. The MISO assembly is pure computation (array
writes, `pgm_read_word`, arithmetic, checksums) — no interrupt-
dependent calls — so it is safe under NMI block. The additional
~50-100µs of NMI blocking is negligible (total goes from ~16ms to
~16.1ms for 33-byte frames).

### What Changes in MHI-AC-Ctrl-core.cpp

**Change 1 — Top of file**: Add `#ifdef ESP8266` block with:
- `#include <c_types.h>` and `#include <ets_sys.h>`
- Safety-net `#ifndef` definitions for `xt_rsil` and `xt_wsr_ps`
- `MHI_GPIO_IN`, `MHI_GPIO_SET`, `MHI_GPIO_CLR` register defines
- `MHI_GET_CCOUNT()` cycle counter macro
- `volatile bool ota_in_progress = false;` flag definition

**Change 2 — After frame-start detection**: Add `#ifdef ESP8266`
block that:
- Computes pin bitmasks from SCK_PIN, MOSI_PIN, MISO_PIN
- Computes timeout in CPU cycles from `max_time_ms`
- Calls `xt_rsil(15)` if `!ota_in_progress`

**Change 3 — The SPI frame exchange loop**: Replace the existing
`for (byte_cnt...)` loop with an `#ifdef ESP8266` version that:
- Captures `start_ccount` via `MHI_GET_CCOUNT()`
- Runs the bit-bang loop using direct GPIO registers and CCOUNT timeout
- Calls `xt_wsr_ps(saved)` on all exit paths (normal and timeout)
- Falls through to original code in `#else` for non-ESP8266 platforms
- Includes a comment in the `#else` path directing ESP32 users to
  https://github.com/hberntsen/mhi-ac-ctrl-esp32

**Change 4 — Header file**: Add `extern volatile bool ota_in_progress;`

### What Does NOT Change

- Frame-start detection (the 5ms SCK-high wait) — runs before NMI block
- MISO frame assembly logic (unchanged, just now runs under NMI block)
- MOSI frame validation and parsing — runs after NMI block
- All setter methods (set_power, set_mode, etc.) — called from ESPHome loop, unrelated

### Why the Frame-Start Detection Is NOT Protected

The `while (millis() - SCKMillis < 5)` loop that detects the inter-frame
gap runs with interrupts enabled. It needs `millis()` and isn't
time-critical — it's just waiting for a 40ms idle period. An NMI during
this phase just slightly delays the detection. This is the only
remaining vulnerability window after the fix.

### OTA Guard

During OTA firmware updates, the ESP8266 receives large bursts of TCP
data. Blocking NMI for 10ms during OTA could cause DMA ring overflow
and slow/stall the transfer. The `ota_in_progress` volatile flag
disables NMI blocking. SPI errors will occur during OTA (acceptable —
no AC control needed while reflashing).

The flag is set/cleared from ESPHome YAML OTA callbacks:
```yaml
ota:
  - platform: esphome
    on_begin:
      then:
        - lambda: |-
            extern volatile bool ota_in_progress;
            ota_in_progress = true;
    on_end:
      then:
        - lambda: |-
            extern volatile bool ota_in_progress;
            ota_in_progress = false;
    on_error:
      then:
        - lambda: |-
            extern volatile bool ota_in_progress;
            ota_in_progress = false;
```

---

## Safety Analysis — Why ~16ms NMI Blocking Is Safe

### WiFi Hardware Buffering

| Layer | Buffer Depth | What It Holds |
|-------|-------------|---------------|
| 802.11 MAC DMA ring | 5-7 descriptors | Raw received frames from radio |
| wDev NMI handler | N/A (runs in ~10-50µs) | Drains DMA → pbuf |
| pbuf RX queue | Heap-limited | Software packet queue for lwIP |

During a ~16ms NMI block (33-byte frame + MISO assembly) at typical home network traffic:
- 0-2 inbound frames arrive (beacons are every ~100ms, API keepalives are sparse)
- 0-2 outbound frames are pending TX completion
- Hardware DMA ring can hold 5-7 frames — no overflow risk

### Protocol Timer Analysis

| Timer | Minimum Value | ~16ms Block Impact |
|-------|--------------|-------------------|
| TCP RTO | 200ms | 8% of minimum — invisible |
| WiFi beacon | 102.4ms (DTIM=1) | 16% overlap chance — buffered in DMA |
| ESP8266 software WDT | ~3.2s | 0.5% of timeout — no risk |
| ESP8266 hardware WDT | ~8s | 0.2% of timeout — no risk |
| 802.11 ACK (SIFS) | 10µs | Hardware-only — CPU not involved |

### Duty Cycle

- Frame: ~16ms protected (NMI blocked, includes MISO assembly + bit-bang)
- Gap: ~34ms free (NMI services normally)
- **Duty cycle: ~32% blocked / ~68% free**

During the ~34ms gap: NMI fires immediately (it's latched), drains all
accumulated DMA descriptors, lwIP processes accumulated pbufs when
`yield()` is called between frames.

### Precedent

FastLED has shipped `xt_rsil(15)` on ESP8266 for WS2812 timing for
years. Their LED output blocks interrupts for similar durations
(~30µs per LED × number of LEDs, commonly 10-30ms total). This is
well-tested in the field with WiFi-connected ESP8266 devices.

---

## The User's Configuration

- **Hardware**: LOLIN/Wemos D1 Mini (ESP8266, 4MB flash)
- **CPU**: 160MHz (configured via `board_build.f_cpu: 160000000L`)
- **ESPHome version**: 2026.3.3
- **Ginkage component version**: v4.x (uses ESPHome codegen, not custom platform)
- **Frame size**: 33 (WF-RAC, includes vanes LR and 3D auto)
- **Connection**: Level-shifted SPI to AC indoor unit CNS connector
- **AC model**: MHI SRK-series split system

The user's ESPHome YAML is at `Y:\esphome\family-room-aircon.yaml`.
The error log is in the project files as `family-room-aircon.logs`.

---

## Implementation Checklist (Completed 2026-04-09)

1. [x] Fork https://github.com/ginkage/MHI-AC-Ctrl-ESPHome
   → https://github.com/michaelp1742/MHI-AC-Ctrl-ESPHome
2. [x] Create branch `nmi-hardened`
3. [x] Edit `components/MhiAcCtrl/MHI-AC-Ctrl-core.h`:
   - Add `extern volatile bool ota_in_progress;` before class definition
4. [x] Edit `components/MhiAcCtrl/MHI-AC-Ctrl-core.cpp`:
   - Add `#ifdef ESP8266` includes/macros block after `#include "MHI-AC-Ctrl-core.h"`
   - Add `volatile bool ota_in_progress = false;` definition
   - Add `#ifdef ESP8266` NMI lock after frame-start detection (covers MISO assembly + bit-bang)
   - Replace the SPI frame exchange `for` loop with NMI-hardened version
   - Keep `#else` fallback with original code for non-ESP8266
   - Add a comment in the `#else` path noting that ESP32 users should
     consider https://github.com/hberntsen/mhi-ac-ctrl-esp32 for a
     hardware SPI solution that eliminates this class of error entirely
5. [x] User updates their YAML (`Y:\esphome\family-room-aircon.yaml`):
   - Point `external_components` to `michaelp1742/MHI-AC-Ctrl-ESPHome@nmi-hardened`
   - Add OTA `on_begin`/`on_end`/`on_error` callbacks
   - `component` log level already set to `ERROR`
6. [x] Test:
   - Flashed via OTA (reliable with old code)
   - First version (NMI block around bit-bang only): ~84% error reduction,
     but new -3 errors appeared from vulnerability window during MISO assembly
   - Second version (NMI block covers MISO assembly + bit-bang): 0 errors
     in 1+ hour of operation
   - OTA update completed successfully
   - 24h WiFi stability: pending

---

## Key Code References

### MHI-AC-Ctrl-core.cpp Structure (loop function)

```
int MHI_AC_Ctrl_Core::loop(uint max_time_ms) {
  // 1. Variable declarations, static frame buffers
  // 2. Frame-start detection (5ms SCK-high wait)     ← NOT protected
  // --- xt_rsil(15) NMI block starts here (ESP8266) ---
  // 3. MISO frame building:                           ← PROTECTED
  //    - doubleframe toggle
  //    - opdata request staging
  //    - command loading (new_Power, new_Mode, etc.)
  //    - checksum calculation
  // 4. SPI frame exchange (bit-bang loop)             ← PROTECTED
  // --- xt_wsr_ps() NMI block ends here (ESP8266) ---
  // 5. MOSI validation (signature, checksum)          ← NOT protected
  // 6. MOSI parsing (status, opdata, error data)      ← NOT protected
  //    - calls m_cbiStatus->cbiStatusFunction() for each changed value
  // 7. return call_counter
}
```

### Pin Configuration (ginkage v4.x)

In v4.x, pins are configurable via YAML and passed to the component.
`SCK_PIN`, `MOSI_PIN`, `MISO_PIN` are `extern int` globals declared
in `MHI-AC-Ctrl-core.h` and defined by the ESPHome codegen. They
hold raw GPIO numbers (e.g., 14, 13, 12), not D-pin aliases. The
GPIO bitmask computation uses `(1 << SCK_PIN)` etc.

### Error Return Values

```c
err_msg_valid_frame = 0      // Success (returns call_counter > 0)
err_msg_invalid_signature = -1
err_msg_invalid_checksum = -2
err_msg_timeout_SCK_low = -3
err_msg_timeout_SCK_high = -4
```

### max_time_ms Value

The platform code (`mhi_platform.cpp:61`) calls `mhi_ac_ctrl_core_.loop(100)`,
so `max_time_ms` is 100. The CCOUNT timeout of `100 * 160000 = 16,000,000`
cycles gives the bit-bang 100ms — well over the ~16ms needed for a
33-byte frame.

---

## Test Results (2026-04-09)

### Before fix (ginkage upstream)
- ~288 errors in 15 minutes (~19.2 errors/min)
- Error distribution: 86% -4 (SCK high), 10% -1 (signature), 4% -2 (checksum), 0% -3

### After fix v1 (NMI block around bit-bang only)
- ~11 errors in 3.5 minutes (~3.1 errors/min)
- ~84% error reduction
- New -3 errors appeared (rising-edge timeout, never seen before)
- Mix of -3 and -4 errors
- Root cause: vulnerability window during MISO assembly between
  frame-start detection and bit-bang loop

### After fix v2 (NMI block covers MISO assembly + bit-bang)
- 0 errors in 1+ hour of continuous operation
- ~100% error elimination
- OTA flash completed successfully
- WiFi connectivity stable throughout

---

## ESP32 Variants — Correct Solutions by Chip

### Chips WITH RMT (ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2)

Use hberntsen's `mhi-ac-ctrl-esp32`:
https://github.com/hberntsen/mhi-ac-ctrl-esp32

This uses hardware SPI slave + RMT peripheral for CS synthesis.
Eliminates the bit-bang entirely. The ginkage codebase should not
be used on these chips — it runs the same fragile software SPI that
causes errors on ESP8266.

The TinyTronics UniversalAircoController (ESP32-S3, pins SCK=21,
MOSI=48, MISO=34) can run hberntsen's component directly. Users
with issue #196 errors should switch to it.

### Chips WITHOUT RMT (ESP32-C2/ESP8684, ESP32-C61)

These lack the RMT peripheral that hberntsen's latest code uses
for CS synthesis. Three options exist, ranked best to worst:

**Option A (best): hberntsen's timer-based CS synthesis.**
Commit 3ddb8cb of hberntsen's repo uses hardware timers and GPIO
interrupts instead of RMT to generate the synthetic CS signal.
Still uses hardware SPI slave for the actual data transfer.
hberntsen explicitly mentions this as the path for no-RMT chips.

**Option B (fallback): FreeRTOS critical section around bit-bang.**
On RISC-V ESP32 variants (C2, C61), use:
```c
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
taskENTER_CRITICAL(&mux);
// bit-bang loop with direct GPIO register access
taskEXIT_CRITICAL(&mux);
```
This blocks task switching and interrupts below the configured
threshold. Since C2/C61 are single-core, this effectively blocks
everything. Same 10ms safety analysis as ESP8266 applies. GPIO
register addresses and cycle counter differ (RISC-V CSRs, not
Xtensa). FreeRTOS task watchdog needs feeding or timeout increase.

**Option C (worst): Unprotected bit-bang (current ginkage code).**
The status quo. Produces constant errors.

### Why Interrupt Blocking Is Wrong for Most ESP32 Chips

For any ESP32 variant with RMT, interrupt-blocking around a
software bit-bang loop is solving the wrong problem. The hardware
SPI slave peripheral handles all bit timing in silicon — no CPU
involvement, no interrupt vulnerability. The RMT synthesises CS
from the SCK gap. Together they eliminate the entire class of
error. Protecting a software bit-bang with interrupt blocking is
like putting a roll cage in a car with a perfectly good road — it
works but addresses the wrong thing.

The ESP8266 is the ONLY chip where interrupt-protected bit-bang is
the correct and only viable solution, because its SPI slave
hardware has a mandatory command/address phase and CS requirement
that are incompatible with the MHI protocol.

---

## Summary: What This Fork Does and Doesn't Do

### Does:
- Fix ESP8266 SPI reliability via `xt_rsil(15)` NMI blocking
- Block NMI from frame-start detection through end of bit-bang exchange
  (covers MISO assembly + SPI bit-bang — eliminates inter-phase vulnerability)
- Replace `digitalRead()`/`digitalWrite()` with direct GPIO registers
- Replace `millis()` timeout with CCOUNT cycle counter
- Add CCOUNT timeout to rising-edge wait (original had no timeout there)
- Protect OTA updates with `ota_in_progress` flag
- Preserve original code for non-ESP8266 via `#ifdef`

### Does not:
- Fix ESP32/S3/C3 errors (those users should switch to hberntsen)
- Change the MHI protocol handling, frame building, or command logic
- Affect the frame-start detection or MOSI parsing phases

### Key references:
- hberntsen HW SPI: https://github.com/hberntsen/mhi-ac-ctrl-esp32
- hberntsen timer CS (no-RMT): commit 3ddb8cb in above repo
- ginkage ESPHome fork: https://github.com/ginkage/MHI-AC-Ctrl-ESPHome
- absalom-muc original: https://github.com/absalom-muc/MHI-AC-Ctrl
- MHI SPI protocol doc: https://github.com/absalom-muc/MHI-AC-Trace/blob/main/SPI.md
- ESP8266 tech ref: esp8266-technical_reference_en.pdf (in project files)
- AN991 SPI app note: AN991.pdf (in project files)
- Issue #196 (ESP32-S3 same errors): https://github.com/ginkage/MHI-AC-Ctrl-ESPHome/issues/196
- Issue #121 (ESP32 same errors): https://github.com/ginkage/MHI-AC-Ctrl-ESPHome/issues/121
