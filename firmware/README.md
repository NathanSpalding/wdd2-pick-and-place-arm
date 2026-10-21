# Firmware

`WDD2_firmware/WDD2_firmware.ino` is the hand-in version (v2.5). Board: ESP32-S3 (N16R8), Arduino-ESP32 core 3.x, USB CDC on boot enabled, 115200 baud.

## Versions

| Version | Used | Base control |
|---|---|---|
| v1.5 (`archive/`) | 12 Oct: encoder calibration, servo comparison, step tests | Continuous trim on all joints |
| v1.8 (`archive/`) | 12–13 Oct: grasp, accuracy, cycles, power tests | Base trim off |
| v2.5 | 16–20 Oct and hand-in | One-shot base trim, 200 ms base smoothing, 60 °/s base cap, waypoint (W) moves |

## Pins and I²C addresses

| Function | Pin / address |
|---|---|
| Servos: base, shoulder, elbow, wrist, gripper | GPIO 11, 12, 13, 14, 16 |
| I²C SDA, SCL (400 kHz) | GPIO 39, 40 |
| Emergency stop sense (HIGH = pressed) | GPIO 41 |
| TCA9548A multiplexer | 0x70 (encoders on channels 0–3) |
| AS5600 encoders | 0x36 |
| INA219 (gripper supply) | 0x40 |

## Calibration

The encoder zeros, directions and pulse fits are constants at the top of the sketch (`ZERO_RAW`, `DIR`, `P0_US`, `K_DEG_US`). They must be measured again if a magnet, horn or encoder mount is moved.

## Serial commands

The full command list is in the comment at the top of the sketch. The main ones:

| Command | Action |
|---|---|
| `J,b,s,e,w` | Joint targets in degrees (streamed by the GUI at 50 Hz) |
| `W,b,s,e,w,v[,id]` | Coordinated move at v °/s; replies `Y,id` when done |
| `G,1` / `G,0` | Close with grasp detection / open |
| `S` / `R` | Software stop / reset |
| `X,j,0|1|2` | Trim loop off, continuous or one-shot for joint j |
| `P`, `T` | Status, timing |

The ESP32 reports `A,...` (joint angles) and `I,...` (gripper current) at 50 Hz, `K,1/0/2` for grasp results and `E,1/0` for the emergency stop.
