# WDD2 report figures: numbers, sources and doubts

Four figures for the report, made only from logged data. Each script is in this folder and writes its PNG into the folder it is run from (300 dpi, house style).

**Running the scripts:** put the source files in one folder and run, for example:

```
set WDD2_DATA=C:\path\to\data
python fig_cycle_timeline.py
```

| Figure | File | Script | Source files |
|---|---|---|---|
| 1 | `test-cycle-timeline.png` | `fig_cycle_timeline.py` | `gui_export_8.7_500g_arm_export_20261013_221739.xlsx` |
| 2 | `test-servo-staircase.png` | `fig_servo_staircase.py` | `8.3_deadband_shoulder.txt`, `8.3_deadband_wrist.txt` |
| 3 | `ctrl-calibration-fit.png` | `fig_calibration_fit.py` | `WDD2_bringup_data_2026-10-09.xlsx` |
| 4 | `test-base-ringing.png` | `fig_base_ringing.py` | `gui_export_8.7_500g_arm_export_20261013_221739.xlsx`, `arm_export_20261017_005005.xlsx` |

## First: the GUI log interval (Section 3.6.6 needs correcting)

The report says the GUI data log is written every 100 ms. **That is wrong for the logs used here.** Both GUI exports were logged every control loop:

| Export | Median interval | 95th / 99th percentile | Notes |
|---|---|---|---|
| 13 Oct, 500 g (8.7) | 22 ms | 36 / 70 ms | nominal 50 Hz |
| 17 Oct, waypoint | 22 ms | 35 / 59 ms | nominal 50 Hz |

The interval is irregular: the Windows Python loop occasionally stalls, giving gaps of up to about 0.2 s during sequences. Longer gaps (up to 29 s) occur outside sequences, for example while exporting.

The logged joint angle is the latest encoder reading the GUI had received. The ESP sends these at 50 Hz, so each value can be up to about 20 ms old.

A suggested wording: "the GUI logs every control loop (nominally 50 Hz; median interval 22 ms)".

An older GUI version logged every 5th loop (10 Hz). That may be where the 100 ms came from.

## Figure 1: one pick-and-place cycle (`test-cycle-timeline.png`)

**Cycle used:** 500 g, counted trial 9 (GUI cycle 11, 13 Oct). It was chosen because its numbers sit near the median of the ten counted 500 g cycles: base overshoot, cycle time (11.90 s), place error (1.4 mm) and holding current. The trial cycle was not used. Trial 8 was avoided because of its unusually high holding current (0.317 A).

**Phases:** taken from the logged target position (`target_x/y/z_mm`) and `holding_cube`, not by eye. Times are from the start of the sequence run:

| Phase | Time (s) |
|---|---|
| Swing to pick (includes the 2 s wait) | 0.00–3.22 |
| Down | 3.22–4.07 |
| Grip (G,1 sent at 4.07) | 4.07–4.50 |
| Up | 4.50–5.20 |
| Swing to place (includes the 2 s wait) | 5.20–8.85 |
| Down | 8.85–9.57 |
| Release (G,0 sent at 9.57) | 9.57–10.01 |
| Up | 10.01–10.70 |
| Return | 10.70–11.90 |

**Numbers for the caption and text:**

| Quantity | Value |
|---|---|
| Cycle time | 11.90 s (GUI cycle report and log agree) |
| Gripper current peak at grasp | 1.13 A |
| Gripper current peak at release | 1.43 A (the opening stroke; higher than at grasp) |
| Gripper holding current (median) | 0.110 A, against the 0.04 A threshold |

Base overshoot, measured past the final target:

| Swing | Measured overshoot | Of which the GUI command itself overshot |
|---|---|---|
| To pick, +45° | 2.11° | 1.97° |
| To place, −45° | 2.44° | 2.23° |
| Return home | 3.97° | 2.27° (the arm was still moving when the sequence ended) |

**Doubts and notes:**
- **The base overshoot in 8.7 mostly came from the GUI's own command.** Across all ten counted 500 g cycles:
  - at the place station, the measured overshoot was 1.9–3.8° (mean 2.6°) and the GUI command overshoot was 1.7–3.9° (mean 2.6°);
  - at the pick station, the measured overshoot was 1.9–3.0° and the command overshoot 1.8–3.1°.

  The GUI's speed and acceleration limiter braked late at the "40 %" speed setting. The ringing after the stop is mechanical. If the report says the 8.7 overshoot is mechanical, it should be corrected.
- **The commanded base line is the angle actually sent to the ESP**, atan2(−tip_y, tip_x) (`BASE_AIM_AT_TIP`). The `base_cmd_deg` column is the model's own base angle, which is 1.36° off because of the model's 7 mm tool offset. The other joints use their `*_cmd_deg` columns.
- **The shoulder shows a one-sample encoder glitch** at about 6.0 s (63.9° → 23.8° → 58.1° → 64.0°). It is plotted as logged and labelled. There are 3 such events in the whole 13 Oct log.

## Figure 2: servo staircase (`test-servo-staircase.png`)

**Method:** each commanded pulse (1 µs steps downward) is paired with the median of the last 10 encoder samples before the next command. Angle change = DIR × (count − start count) × 360/4096. The dotted lines are the ideal servo response from the calibration slope K.

**Series:** the shoulder (worst case), plus the **wrist instead of the elbow**. The elbow's stepping-down log covers only 6 µs (1582 → 1576 µs) and has a 1-count creep before the reversal, so it does not give a clean staircase.

| | Shoulder (K = −0.1292 °/µs) | Wrist (K = −0.1361 °/µs) |
|---|---|---|
| Start pulse | 1558 µs | 1548 µs |
| Reversal deadband | **28 µs = 3.62°** (first move at 1530 µs) | **9 µs = 1.23°** (first move at 1539 µs) |
| Measured jumps (encoder) | 0.48° at 28 µs, 0.79° at 34 µs | 0.35° at 9 µs, 0.44° at 13 µs, 0.26° at 16 µs |
| Jump spacing | 6 µs | 4 µs, then 3 µs |
| At the end of the log | 1.32° moved after 38 µs (ideal 4.91°) | 1.14° moved after 20 µs (ideal 2.72°) |

The encoder resolves single counts of 0.088°. Single-count blips are visible: the shoulder at 24 µs, and the wrist's creep at 7–8 µs.

**Against `8.3_servo_deadband.csv` (Table 18):** the deadbands agree exactly (shoulder 28 µs / 3.618°, wrist 9 µs / 1.225°), and so do the step spacings (shoulder 6 µs, wrist about 3.5 µs). Two points to word carefully:
- **The CSV's step size in degrees is the spacing in µs × the slope, not the measured jump.** For the shoulder the measured second jump is 0.79°, which matches the CSV's 0.775°. For the wrist, the measured jumps are 0.26–0.44°, smaller than the CSV's 0.476°.
- **The reversal deadband in degrees is the command-equivalent angle (µs × |K|), not an angle the joint moved.**

## Figure 3: calibration fit (`ctrl-calibration-fit.png`)

**Data:** the bring-up calibration sweeps (9 Oct). Each row is a commanded pulse and the settled encoder angle. Points pressed against a mechanical stop are excluded, as in the original fit. The fits were recomputed by least squares and reproduce the published values. In panel (b), filled points were reached by raising the pulse and hollow points by lowering it, which shows that the large residuals are hysteresis.

| Joint | Points (excluded at stops) | K (°/µs) | P0 (µs) | RMS residual | Largest residual | R² |
|---|---|---|---|---|---|---|
| Shoulder | 14 (3) | −0.1292 | 1558.3 | 1.60° | 2.96° | 0.9976 |
| Elbow | 22 (0) | −0.1358 | 1562.5 | 0.41° | 0.90° | 0.99997 |
| Wrist | 15 (0) | −0.1361 | 1548.2 | 1.26° | 3.51° | 0.99967 |
| Base | — | +0.1276 | 1623 | 0.45° (not checked) | 0.74° (not checked) | — |

**Doubts:**
- **The base calibration sweep is not in any file I have.** The base was calibrated after the bring-up night, and only its summary (RMS 0.45°, max 0.74°) survives in the day-1 notes. Those two numbers cannot be re-checked here, and no base points were reconstructed.
- **"Measured angle" here is the joint's own encoder, not an independent reference.** The ±0.5° band in panel (b) is the uncertainty of the 8.2 reference (iPhone level), drawn for scale only. The caption should not imply these points were measured against that reference.
- **The shoulder residuals are dominated by hysteresis:** points reached by raising the pulse sit mostly high (−0.41 to +2.57°), and points reached by lowering it sit mostly low (−2.96 to +1.34°). A single straight line cannot fit both directions better than about ±3°. This is why the encoder trim loop is needed.
- **The elbow sheet has three identical rows** at 1000 µs (76.46°). They may be repeated entries rather than three readings. They are kept in the fit, as in the original.

## Figure 4: base ringing (`test-base-ringing.png`)

**Data supports it:** the GUI log is about 22 ms (50 Hz), not 100 ms, which is fine for a 3 Hz ring (about 15 samples per cycle).

**Profiles compared:**

| Profile | Data | Settings |
|---|---|---|
| Original | the ten counted 500 g cycles of 8.7 (13 Oct) | GUI streaming, 80 ms S-curve, continuous base trim |
| Final | the four arrivals with the 500 g cube held (17 Oct, waypoint export) | waypoint mode, 200 ms S-curve, 60 °/s cap, one-shot trim |

All traces are drawn thin; the arrival with the median ring is drawn bold. t = 0 is the first sample with the measured base within 1° of the −45° target. Error = measured − target, and negative means past the target.

| Measure | Original (n = 10) | Final (n = 4) |
|---|---|---|
| Overshoot past target | 2.6° mean (1.9–3.8°) | 0.2° mean (0.0–0.4°) |
| Peak-to-peak, 0.5–2.0 s after arrival | 1.8° mean (1.3–3.0°) | 1.3° mean (1.2–1.5°) |
| Settling time to within ±0.3° of the final value | 2.3 s mean (1.3–3.0 s) | 2.5 s mean (2.3–3.0 s) |
| Final error, 2.5–3.0 s (short of target) | 0.63° (0.36–0.87°) | 0.36° (0.18–0.54°) |

**Checking the report's statements:**

| Report states | What the data shows |
|---|---|
| "Overshoot of 1.2° to 2.3°" | **Correct it.** In the 13 Oct 8.7 runs the measured overshoot past the −45° target was 1.9–3.8° (mean 2.6°), almost all of it from the GUI's own command overshoot (1.7–3.9°). The 1.2–2.3° came from an earlier analysis that measured against the model's base angle, which is offset. |
| "A ring of about 1.5° peak-to-peak, reduced to 0.6°" | **Partly right; say where each number comes from.** The 0.6° (0.62° twice) is from the Serial Monitor test: `W` moves of the extended arm with 500 g at 48 °/s, swing measured over 3 s with `O,0,3`. In the full GUI sequence the final profile still rings by 1.2–1.5° peak-to-peak (0.5–2.0 s window), against 1.3–3.0° originally. |
| "Some cycles still ringing by 1.4°" | **Confirmed:** the final-profile arrivals ring by 1.2–1.5°. |

**What the final profile actually changed:** it removed the overshoot (2.6° → 0.2°) and the large first swing, and reduced the final error (0.63° → 0.36°). **It did not shorten the settling time** (about 2.3–2.5 s in both), and the residual ring of about ±0.6° remains. That fits the conclusion that the remaining ring is mechanical: play plus a compliant servo.

**Doubts:**
- **Several things changed between the two profiles at once**: GUI streaming became waypoint mode, the S-curve went from 80 to 200 ms, the speed cap was added, and the trim became one-shot. The difference cannot be credited to the S-curve alone.
- **Only four final-profile arrivals were logged with the cube held.** Other 17 Oct cycles stopped on grasp failures, and the 17 Oct streaming run at 200 ms has no clean left-station arrivals in its export.
- **One final-profile window has a 197 ms gap in the log**, which shows as flat steps in the thin blue traces.
