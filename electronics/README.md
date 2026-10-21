# Electronics

- KiCad project: the schematics as drawn for the build.
- `schematic_redrawn.pdf` / `.tex`: the single-sheet circuit from Appendix B of the report (circuitikz source).
- `BOM.xlsx` / `BOM.csv`: every part with quantity, price, supplier link and printed-part masses.

## Power

| Rail | Voltage | Servos |
|---|---|---|
| 1 | 8.0 V | Shoulder (DS5180) |
| 2 | 6.7 V | Base and elbow (DS3240) |
| 3 | 6.5 V | Wrist (DS3225) and gripper (MG996R) |
| Logic | 5 V | ESP32-S3 and sensors |

The 12 V supply passes through a 15 A fuse and a relay held by the latching emergency stop. Pressing the stop cuts all three servo rails; the logic stays on.

**Fit the relay flyback diode with its cathode to pin 86.** A reversed diode shorts the supply at power-up.
