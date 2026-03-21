
---


# ESP32C3 Bluetooth HID to PS2 Adapter

A converter based on ESP32C3 & ESP‑IDF V5.5.0.
It converts Bluetooth keyboard and mouse signals into standard PS2 protocol for two PCs.

## Features
- Bluetooth keyboard/mouse to 2× PS2 keyboard + 2× PS2 mouse
- 4 PS2 channels share one timer for efficiency
- I2C LCD1602 status display
- Bluetooth pairing code display, battery level, lock status, unknown command debug

## Hardware Pinout

### PS2 Ports
| Function | GPIO |
|----------|------|
| Key1 CLK | 19 |
| Key1 DAT | 13 |
| Mouse1 CLK | 12 |
| Mouse1 DAT | 18 |
| Key2 CLK | 2 |
| Key2 DAT | 3 |
| Mouse2 CLK | 10 |
| Mouse2 DAT | 6 |

### LCD1602 I2C
| Pin | GPIO |
|-----|------|
| SCL | 5 |
| SDA | 4 |

## LCD Display Format
Line 1: PC1 status  
Line 2: PC2 status

Position definition:
1: `>` = active host
2: S = keyboard ready, s = not ready / error
3: 1 = NumLock on, ← = off
4: A = CapsLock on, a = off
5: - = ScrollLock on, | = off
6: K = BT keyboard connected, k = disconnected
7: M = BT mouse connected, m = disconnected
8~13: Bluetooth pairing code (lower priority than error codes)
Pos12: K/M + 4 hex = unknown incoming command
Line2 6~8: keyboard battery percentage(Work not well)
Line2 9~11: mouse battery percentage(Work not well)

## Development Environment
- ESP-IDF Version: 5.5.0
- Drivers from:
  - Reference/ps2 (tested, shared timer)
  - Reference/lcd1602 (I2C, verified)

## Build & Flash
```bash
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
