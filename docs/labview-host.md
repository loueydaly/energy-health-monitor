# LabVIEW Host Application (`main.vi`)

The PC side of the Energy Health Monitor is a single LabVIEW VI (`main.vi`) that is both the
**operator HMI** and a **built-in V/I waveform simulator**. It synthesises reference
voltage/current signals, streams them through the ESP32 bridge to the STM32 processing unit
over CAN, and displays the returned energy-quality metrics live.

## Role in the system

```
 Generate_Viv ─► Build_Frame.vi ─► VISA Write ─► USB UART ─► ESP32 ─► CAN 0x100 ─► STM32
 (V/I sim)                                                                 │
 UI gauges ◄────────── parse ◄────────────── VISA Read ◄────────────────────┘ (0x200 metrics)
```

1. **Generate_Viv** synthesises one V/I sample pair from the front-panel waveform settings.
2. **Build_Frame.vi** wraps the pair in the 14-byte framed packet (same format the ESP32
   bridge expects — see the [main README](../README.md#uart-framing-labview--esp32)).
3. The frame is written to the USB serial port; the STM32 processes the stream and answers
   with quality-metric frames (CAN ID `0x200`), which the bridge relays back over UART.
4. `main.vi` decodes the replies and updates the gauge, grade, sub-scores and alert LEDs.

## Front panel

### Controls

| Control | Typical value | Description |
|---|---|---|
| **VISA Resource** | `COM6` | USB serial port of the ESP32 bridge |
| **Baud Rate** | `115200` | Serial baud rate (matches `Serial.begin(115200)` in the ESP32 sketch) |
| **CAN_ID** | `100` (0x100) | CAN identifier stamped into each transmitted frame |
| **Sample Period (ms)** | `5` · `20` · `50` | Loop delay between generated sample frames (all three rates are exercised in the captures below) |
| **V_Amplitude** | `230` | Simulated voltage amplitude (V) |
| **I_Amplitude** | `10` | Simulated current amplitude (A) |
| **Frequency** | `50` | Grid frequency of the simulated waveforms (Hz) |
| **Phase** | `0,2` | V–I phase offset in radians (European decimal comma = 0.2) — sets the test power factor |
| **stop** | — | Ends the loop; the VISA port is closed afterwards |

### Indicators

| Indicator | Description |
|---|---|
| **Voltage Chart / Current Chart** | Time plots of the generated & transmitted V/I waveforms |
| **Last Frame (Hex)** | The 14-byte packet most recently written to the ESP32 (shown as a hex byte array) |
| **Frames Sent** | Counter of transmitted frames |
| **Comm LED** / **status** | Serial-link activity / overall VI status |
| **Received Frame (Hex)** | Raw bytes read back from the ESP32: bridge status bytes (`AA` …) and relayed `0x200` metric frames |
| **Overall Energy Quality %** | 0–100 gauge with colour-coded quality zones |
| **Energy Grade** | Text grade decoded from the STM32 quality scorer (e.g. `"CRITICAL (F)"`) |
| **Raw PF** | Power factor as returned in the `0x200` metrics frame |
| **Voltage Quality % / PF Efficiency % / Circuit Safety %** | The three sub-scores of the quality metric |
| **Last ACK Byte** | Last bridge status byte (`AA` = ACK; full code table in the [main README](../README.md#uart-framing-labview--esp32)) |
| **System OK / Overcurrent Alert / Voltage Sag-Surge** | Alert LEDs decoded from the metrics flags byte |
| **error out** | Standard LabVIEW error cluster (see [Error handling](#error-handling)) |

### Screenshots

Steady-state streaming at **5 ms** sample period — clean 230 V / 10 A sine waves on both
charts, `Frames Sent` ≈ 2905, `error out` code 0, Comm LED active:

![LabVIEW front panel – 5 ms steady-state streaming](images/labview-front-panel-5ms.png)

Qualification run at **50 ms** — charts filling in from zero while the quality panel is live
(`Received Frame (Hex)` shows the `AA` bridge ACK; `Last ACK Byte` = `AA`):

![LabVIEW front panel – 50 ms streaming run](images/labview-front-panel-run.png)

Transient/impulse test at **20 ms** — sharp V and I spikes on the charts, full sub-score
bars, gauge needle swung to the far zone while the grade reads `"CRITICAL (F)"`:

![LabVIEW front panel – 20 ms transient test](images/labview-front-panel-transient.png)

## Block diagram

![LabVIEW main.vi block diagram](images/labview-block-diagram.png)

Per iteration of the timed loop (governed by **Sample Period (ms)**):

1. **Generate_Viv** produces the next V/I sample pair from `V_Amplitude`, `I_Amplitude`,
   `Frequency`, `Phase`; the samples feed the **Voltage** and **Current Charts**.
2. **Build_Frame.vi** receives the pair, `CAN_ID` and builds the 14-byte framed packet.
3. **VISA Write** sends the packet; the returned bridge status byte feeds **Last ACK Byte**
   and the **Comm LED**. Each successful send increments **Frames Sent**.
4. **VISA Read** polls the port for the 14-byte reply window and displays the raw bytes on
   **Received Frame (Hex)**; parsed `0x200` payloads drive the quality gauge, grade,
   sub-score bars and alert LEDs.

   The read uses the standard VISA Read node — resource in, byte count, read buffer and
   return count out:

   ![VISA Read node](images/labview-visa-read.png)

5. The loop sleeps for `Sample Period (ms)` and repeats. On **stop**, `VISA Close` releases
   the port.

Serial configuration (`VISA Configure Serial Port`) runs once before the loop with the
**VISA Resource** and **Baud Rate** controls (8 data bits, no parity, 1 stop bit).

### Captured examples

Live transmit frames (`Last Frame (Hex)`) — complete 14-byte packets destined for
CAN ID `0x0100`, captured at 50 ms and 5 ms respectively:

```
02 01 00 08 71 69 8B B0 F7 4B FE BF 11 03
02 01 00 08 75 BB 86 B3 F7 4B FE 3F 8F 03
└┬┘└──┬──┘└┬┘└───── 8 data bytes ─────┘└┬┘└┬┘
 STX  CAN ID DLC        payload          CRC ETX
```

On the return path, `Received Frame (Hex)` shows the bridge status byte (`AA` = ACK)
followed by the STM32 quality frame (ID `0x0200`) being relayed — often partially, since
the polling read can return mid-frame while streaming:

```
AA 02 02 00 08 4B E6 3D 7E 79 2D 0A
└┬┘└──────── relayed 0x0200 frame ────────┘
 ACK
```

## Error handling

The `error out` cluster may show code **1073676294** from *VISA Read in main.vi*. This is
LabVIEW's standard **VISA timeout** — the polling read found no bytes pending between
replies. The steady-state 5 ms capture above shows `code 0` (every read returned data),
while the slower-rate captures show the timeout between replies; both behaviours are
normal while streaming and are not faults.
