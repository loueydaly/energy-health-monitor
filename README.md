# Energy Health Monitor

Professional AC power-quality monitoring system: an **STM32F446RE** runs real-time energy-processing algorithms (per IEEE 1459-2010 / IEC 62053-22 / IEC 61000-4-30), an **ESP32** bridges it to a PC over UART, and a **LabVIEW** application (`main.vi`) provides the operator HMI plus a built-in V/I waveform simulator.

```
┌────────────┐   USB UART    ┌─────────────┐    CAN 500 kbps    ┌──────────────────────┐
│  LabVIEW   │  115200 bps   │    ESP32    │  ────────────────► │  STM32F446RE         │
│ (HMI + sim)│ ◄───────────► │  (bridge)   │  ◄──────────────── │  EnergyProcessing    │
└────────────┘  framed 14-B  └─────────────┘   IDs 0x100/0x200  │  FreeRTOS + FPU      │
                                                                └──────────────────────┘
```

## Repository layout

```
energy-health-monitor/
├── docs/
│   ├── labview-host.md        # LabVIEW main.vi documentation (UI + data-flow walkthrough)
│   └── images/                # LabVIEW screenshots (block diagram + front panel)
├── esp32/
│   └── code_ESP32/            # Arduino sketch (UART ⇄ CAN bridge)
│       └── code_ESP32.ino
└── stm32/
    └── EnergyProcessing/      # STM32CubeIDE project (energy processing firmware)
        ├── Core/            # Application code (EnergyProcessing.c is the core library)
        ├── Drivers/         # STM32F4 HAL + CMSIS
        ├── Middlewares/     # FreeRTOS (CMSIS-RTOS v2)
        ├── EnergyProcessing.ioc
        ├── STM32F446RETX_FLASH.ld / STM32F446RETX_RAM.ld
        └── ...
```

Build output (`Debug/`, `Release/`, `*.o`, …) is intentionally excluded — see `.gitignore`.

---

## STM32 firmware — `stm32/EnergyProcessing/`

**Target:** STM32F446RE @ 180 MHz (Cortex-M4F, hardware FPU) · **RTOS:** FreeRTOS via CMSIS-RTOS v2

The `EnergyProcessing` library (`Core/Src/EnergyProcessing.c`) implements:

| # | Algorithm | Standard / method |
|---|-----------|-------------------|
| 1 | True RMS (windowed time-domain) | IEEE 1459 |
| 2 | Active power (instantaneous p·i) | IEEE 1459 |
| 3 | Reactive power (power triangle) | IEEE 1459 |
| 4 | Power factor (numerically safe) | IEEE 519 thresholds |
| 5 | Energy accumulation | IEC 62053-22 Class 1 (±1 %) |
| 6 | Grid frequency via zero-crossing | — |
| 7 | Crest factor | — |
| 8 | THD estimation | IEEE 519 limits |
| 9 | Load classification (resistive / inductive) | — |
| 10 | Anomaly detection (over/under-voltage, overload, sudden change, frequency deviation) | IEC 60038 / IEC 60947-2 |
| 11 | Cost & CO₂ tracking | configurable tariffs |
| 12 | Moving-average / EMA noise filtering | — |

- **ProcessingTask** (above-normal priority): consumes V/I samples from a message queue, updates the 100-sample window (500 ms @ 200 Hz), then runs anomaly checks and the quality scorer.
- **CommTask** (normal priority): publishes quality metrics on CAN.

### CAN protocol (STM32 side)

**RX — ID `0x100`** — raw sample pair (from the acquisition chain):

| Byte | Content |
|------|---------|
| 0–1  | Voltage sample, int16 big-endian, ÷100 → volts |
| 2–3  | Current sample, int16 big-endian, ÷100 → amps |

**TX — ID `0x200`** — quality metrics (DLC 8):

| Byte | Content |
|------|---------|
| 0–1  | Quality score ×100, uint16 big-endian |
| 2    | Voltage quality (0–100) |
| 3    | Power-factor quality (0–100) |
| 4    | Current quality (0–100) |
| 5    | Grade: 3 = excellent · 2 = good · 1 = fair · 0 = poor |
| 6    | \|PF\| × 100 |
| 7    | Flags: bit0 valid · bit1 overcurrent · bit2 voltage deviation |

Score weighting: 40 % voltage · 40 % power factor · 20 % current.

### Build / flash

1. Open **STM32CubeIDE** → *File → Open Projects from File System…* → select `stm32/EnergyProcessing/` (or *Import → Existing Projects into Workspace*).
2. Build (**Ctrl+B**) — the `Debug/` folder is recreated automatically.
3. Flash with ST-LINK (**F11** or *Run → Debug*). A ready-made launch config (`EnergyProcessing Debug.launch`) is included.

---

## ESP32 bridge — `esp32/code_ESP32/`

Bridges the LabVIEW host (USB serial, 115200 bps) to the CAN bus (**TWAI**, 500 kbps, GPIO 4 = RX, GPIO 5 = TX) and forwards traffic in both directions.

### UART framing (LabVIEW ⇄ ESP32)

Fixed **14-byte** frames:

| Byte | Content |
|------|---------|
| 0    | STX = `0x02` |
| 1–2  | CAN identifier, 11-bit standard, big-endian |
| 3    | DLC (payload length, max 8) |
| 4–11 | Payload (8 bytes, zero-padded) |
| 12   | XOR checksum of bytes 1–11 |
| 13   | ETX = `0x03` |

**Status replies** (single byte, ESP32 → LabVIEW):

| Byte | Meaning |
|------|---------|
| `0xAA` | ACK — frame transmitted / acknowledged on CAN |
| `0xEE` | NACK — checksum error |
| `0xF1` | NACK — no ACK from the CAN node |
| `0xF2` | NACK — bus error |
| `0xF3` | Critical — bus-off (recovery auto-initiated) |
| `0xF5` | NACK — TX buffer full / driver error |

The bridge also watches TWAI alerts and automatically restarts the driver after bus recovery.

### Build / flash

1. Open `code_ESP32.ino` in the **Arduino IDE** (or arduino-cli / PlatformIO).
2. Install the ESP32 board package; the sketch uses the bundled `driver/twai.h` (ESP-IDF TWAI).
3. Select your ESP32 board and upload.

---

## LabVIEW host — `docs/`

The PC application `main.vi` is the operator HMI **and** a V/I waveform simulator (`Generate_Viv`) that drives the whole pipeline with configurable test signals (`V_Amplitude`, `I_Amplitude`, `Frequency`, `Phase`) — then decodes the returned `0x200` quality frames into live gauges and alerts.

### Block diagram

![LabVIEW main.vi block diagram](docs/images/labview-block-diagram.png)

### Front panel — live streaming

![LabVIEW front panel – live streaming](docs/images/labview-front-panel-run.png)

### Front panel — transient test

![LabVIEW front panel – transient test](docs/images/labview-front-panel-transient.png)

Controls, indicators, per-iteration data flow, captured frame examples and error-code notes: **[docs/labview-host.md](docs/labview-host.md)**.

---

## System at a glance

- **Electricity metering** accuracy class: IEC 62053-22 Class 1 (±1 %)
- **Window:** 100 samples × 5 ms = 500 ms (25 grid cycles @ 50 Hz, exceeds IEC 61000-4-30's 200 ms minimum)
- **Nominal grid:** 230 V / 50 Hz (European; thresholds configurable in `EnergyProcessing.h`)
- **Memory footprint:** ~1.5 KB RAM · ~8 KB flash (processing library)
- **Update rate:** ~100 ms

## Author

Created by **louey**, Aug 2026.
