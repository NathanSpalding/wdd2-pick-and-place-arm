# WDD2 testing, day 2 (13 October 2026)

Chapter 8 test results for the WDD2 4-DOF pick-and-place arm (Mechatronic Project 478), covering tests 8.6, 8.7 and 8.8. For each test this file gives the method, the data and the result. The raw data is in `csv/` and `raw/` next to this file, and the worked sheets with formulas are in `WDD2_Test_Log.xlsx`.

## 1. Summary

| Req. | What | Test | Result | Key number |
|---|---|---|---|---|
| R12 | Positioning ±10 mm | 8.6 | **PASS** | Accuracy 3.1–6.4 mm and repeatability 4.7–7.0 mm over 9 targets; worst single approach 9.1 mm |
| R13 | Pick-and-place 9/10, each ≤ 15 s | 8.7 | **PASS** | 50 g 10/10, 500 g 10/10; cycle time 11.7–12.5 s; place error ≤ 8.0 mm |
| R1 | Reach ≥ 300 mm | 8.7 | **PASS** | Pick and place stations both at r = 300 mm |
| R3 | Payload 500 g | 8.7 | **PASS** | 500 g carried and placed in 15/15 cycles |
| R2 | Hold 500 g at full extension | 8.8 | **PASS** | Held for 5 min with all joints horizontal; tip sag 15 mm |
| R5 | Rails within servo ratings | 8.8 | **PASS** | Largest dip 0.34 V (shoulder rail, 7.78 V); no rail rose above its idle value |
| R4 | Servo temperatures | 8.8 | **PASS** | Hottest servo 54.5 °C (elbow, after the 5 min extended hold) against an assumed 60 °C limit |
| R6 | Fuse and protection | 8.8 | **PASS** | Fuse intact after all tests |
| R7 | E-stop cuts servo power | 8.8 | **PASS** | 10/10 presses cut every servo rail to 0 V; the relay also cut power with the ESP signal wire disconnected |

Together with day 1 (R8, R9, R10, R11), all 13 tested requirements pass.

## 2. Test set-up common to all tests

- **Arm and firmware:** as on day 1 (`WDD2_firmware.ino` on the ESP32-S3; 200 Hz loop; AS5600 encoders through a TCA9548A; INA219 on the gripper supply). Encoder zeros ZERO_RAW = {1055, 117, 645, 444}, DIR = {+1, −1, −1, −1}.
- **Trim loop:** on for the shoulder, elbow and wrist (KI {2, 2, 2} °/s per °, deadband 0.3°, clamp ±5°). **Off for the base:** in the day-1 step test the base trim kept a ±0.44° hunt going through the play at the horn; with it off every base hold was still and within 0.3°.
- **Commands used:**
  - `W,b,s,e,w,v` moves all four joints together to the given angles (°) at v °/s, with an S-curve profile.
  - `P` prints target, command, encoder angle, trim and pulse for each joint.
  - `G,1` closes the gripper with grasp detection: after 450 ms, the median of 40 current samples must exceed 0.04 A. The firmware then reports `K,1` (grasp) or `K,0` (missed).
  - `G,0` opens the gripper.
- **GUI:** `ui_app.py` (Viser), which streams joint targets to the ESP at 50 Hz. It runs saved sequences and writes a cycle report (cycle time, gripper current) and a 50 Hz data log, both exported to Excel.
- **Target mat:** `WDD2_target_mat_A4x4.pdf`, printed at 100 % on four A4 sheets (566 × 392 mm), taped together and fixed to the table. The arm base is fixed to the table, with the mat's base-axis cross under the base axis and the 0° line under the arm at base 0°.
- **Mat conventions:**
  - Robot frame: x forward, y to the left.
  - Base angle: + to the right, seen from behind the arm.
  - Error components:
    - radial: + beyond the target, − short of it;
    - lateral: + right of the target ray, − left of it, seen from the base looking out along the ray.
- **Instruments:**
  - steel rule (1 mm);
  - multimeter;
  - Fluke VT02 visual thermometer (case temperatures);
  - iPhone Measure level (±0.5°);
  - iPhone video for timing and meter readings.

## 3. Test 8.6: positioning accuracy and repeatability (R12)

### 3.1 Method

- **Tool point:** a pen held in the gripper, wedged in a bottle cap and taped so that it couldn't slide or twist in the jaws. The pen tip is 185 mm from the wrist axis.
- **Targets:** T1–T8 at r = 250 and 350 mm, at base angles ±35° and ±50°. T9, the repeatability target, is at r = 300 mm, +45°. All targets are outside ±30° on the base.
- **Joint angles:** each target was commanded as joint angles (`W`), solved by inverse kinematics with the pen tip as the tool and the pen vertical. The base angle equals the mat angle. Each approach went:
  1. from home (0, 72, −82, −80);
  2. to a via point 50–70 mm above the target;
  3. down to the target, with the pen tip hovering about 6 mm above the mat.

  The joint commands are in `csv/8.6_joint_commands.csv`.
- **Reading:** at the target, the radial and lateral offsets of the pen tip from the cross were read by eye against the 2 mm rings, to the nearest 1 mm. The arm then returned home before the next approach.
- **Repetitions:** 10 approaches at T1–T8 and 30 at T9.
- **Metrics (ISO 9283 style):**
  - Accuracy is the distance from the target to the centroid of the landing points.
  - Repeatability is the mean distance of the landing points from their centroid, plus 3 standard deviations.
- **Pass criterion:** accuracy ≤ 10 mm at every target.

### 3.2 Results

| Target | r (mm) | Angle (°) | n | Mean radial (mm) | Mean lateral (mm) | Accuracy (mm) | Repeatability (mm) | Worst single (mm) | Result |
|---|---|---|---|---|---|---|---|---|---|
| T1 | 250 | −50 | 10 | −3.0 | +2.4 | 3.84 | 4.67 | 6.3 | PASS |
| T2 | 250 | −35 | 10 | −3.1 | +0.5 | 3.14 | 5.41 | 6.0 | PASS |
| T3 | 250 | +35 | 10 | −4.2 | +0.1 | 4.20 | 6.11 | 8.2 | PASS |
| T4 | 250 | +50 | 10 | −5.1 | +1.3 | 5.26 | 5.42 | 8.9 | PASS |
| T5 | 350 | −50 | 10 | −1.8 | +4.3 | 4.66 | 5.32 | 8.2 | PASS |
| T6 | 350 | −35 | 10 | −0.8 | +3.2 | 3.30 | 5.52 | 5.4 | PASS |
| T7 | 350 | +35 | 10 | −3.7 | +2.1 | 4.25 | 4.96 | 7.8 | PASS |
| T8 | 350 | +50 | 10 | −5.7 | +3.0 | 6.44 | 6.80 | 8.5 | PASS |
| T9 | 300 | +45 | 29 | −5.3 | +0.2 | 5.31 | 7.03 | 9.1 | PASS |

T9 approach 10 has no lateral reading, so it is excluded (n = 29).

**Result: PASS.** The worst accuracy is 6.44 mm (T8), the worst repeatability is 7.03 mm (T9), and every single approach landed within 9.1 mm of its target.

### 3.3 Observations

- **The pen tip always landed short.** The mean radial error is negative at every target (−0.8 to −5.7 mm), and it is larger at the far targets on the right-hand side (T4, T8). That pattern points to a fixed geometric error, such as link lengths or mat placement, not random scatter.
- **The radial error grew over successive approaches.** At T9 it went from −1.4 mm over the first approaches to −8.2 mm over the last ones (30 approaches). Checks after the T9 set:
  - `P` at T9 showed every joint within 0.44° of its command.
  - Pushing on the tip by hand moved it about 3.7 mm. The encoders saw the movement, with some stick-slip.
  - A link-angle check with the iPhone level (30.5°, 29.8°, 29.7°) agreed with the encoders, so the encoder magnets had not slipped.

  The joint offsets explain about 2 mm of the shortfall. The rest of the growth is not explained by the joint angles.
- **The lateral error is small:** mean lateral error +0.1 to +4.3 mm, always to the right of the ray.
- **Sensitivity of the tip to a 1° joint error** (from the kinematic model):

| Target | Base | Shoulder | Elbow | Wrist |
|---|---|---|---|---|
| T9 | 5.2 mm | 4.5 mm | 4.6 mm | 3.2 mm |
| T8 | 6.1 mm | 5.3 mm | 4.4 mm | 3.2 mm |

  The joint errors seen at T9 (≤ 0.44°) are therefore worth up to about 2 mm at the tip.
- T10 and T11, the 500 g versions of T9 and T7, were not run.

## 4. Test 8.7: pick-and-place cycles (R13, R1, R3)

### 4.1 Method

- **Stations:**
  - pick at the right station: r = 300 mm, +45° on the mat, robot (212.1, −212.1) mm;
  - place at the left station: r = 300 mm, −45°, robot (212.1, 212.1) mm.

  The arm turns 90° on the base between the two stations, and both stations are 300 mm from the base axis (R1).
- **Object:** the 40 mm cube loaded to the 50 g and 500 g payloads used in test 8.5.
- **Sequence** (saved GUI sequence `test87_side_transfer`, run in sequence mode):
  1. Home (255, 7, 97) mm.
  2. Above the right station (z = 100 mm), then wait 2 s.
  3. Down to the right station (z = 25 mm), and grab.
  4. Up to above the right station.
  5. Across to above the left station, then wait 2 s.
  6. Down to the left station, and release.
  7. Up to above the left station, then Home.

  The 100 mm approach height clears the 40 mm cube. The 2 s waits let the base settle before each descent.
- **Gripper:** the gripper stays vertical throughout. The grab uses grasp detection.
- **Measurements per cycle:**
  - picked and placed upright (Y/N);
  - place error: the radial and lateral offset of the cube centre from the place mark, in mm, by rule;
  - cycle time, from the GUI cycle report;
  - gripper holding current, from the INA219 in the GUI report;
  - whether a visible oscillation occurred just before the release.
- **Runs:** a trial cycle, then 10 counted cycles with 50 g and 10 with 500 g. Five extra 500 g cycles were then run to check whether the oscillation affects placement. Cycles stopped by hand before completion are not counted.
- **Pass criteria:**
  - at least 9 of 10 successful cycles per payload, each completed within 15 s;
  - success means picked, carried and released upright, with a place error ≤ 10 mm.

### 4.2 Results: 50 g

| Cycle | Placed upright | Radial (mm) | Lateral (mm) | Place error (mm) | Cycle time (s) | Holding current (A) | Oscillation before drop |
|---|---|---|---|---|---|---|---|
| 1 | Y | +2 | −2 | 2.8 | 11.71 | 0.127 | |
| 2 | Y | +3 | −2 | 3.6 | 11.80 | 0.129 | |
| 3 | Y | +2 | −2 | 2.8 | 11.84 | 0.105 | |
| 4 | Y | 0 | −1 | 1.0 | 12.06 | 0.115 | |
| 5 | Y | +3 | 0 | 3.0 | 11.85 | 0.118 | |
| 6 | Y | +4 | −3 | 5.0 | 11.83 | 0.127 | yes |
| 7 | Y | +4 | −2 | 4.5 | 12.41 | 0.122 | yes |
| 8 | Y | +2 | −1 | 2.2 | 11.95 | 0.125 | |
| 9 | Y | 0 | −1 | 1.0 | 12.25 | 0.119 | |
| 10 | Y | +2 | −1 | 2.2 | 12.49 | 0.103 | |

10/10 successful. Place error 1.0–5.0 mm (mean 2.8 mm). Cycle time 11.71–12.49 s (mean 12.02 s).

### 4.3 Results: 500 g

| Cycle | Placed upright | Radial (mm) | Lateral (mm) | Place error (mm) | Cycle time (s) | Holding current (A) | Oscillation before drop |
|---|---|---|---|---|---|---|---|
| 1 | Y | −4 | +4 | 5.7 | 12.42 | 0.131 | yes |
| 2 | Y | 0 | +2 | 2.0 | 11.97 | 0.157 | |
| 3 | Y | 0 | +8 | 8.0 | 11.88 | 0.139 | yes |
| 4 | Y | +1 | +2 | 2.2 | 12.44 | 0.145 | |
| 5 | Y | 0 | +4 | 4.0 | 11.89 | 0.136 | |
| 6 | Y | +3 | +3 | 4.2 | 12.22 | 0.141 | yes |
| 7 | Y | +4 | +4 | 5.7 | 12.10 | 0.138 | yes |
| 8 | Y | −2 | +2 | 2.8 | 12.30 | 0.317 | yes |
| 9 | Y | −1 | +1 | 1.4 | 11.90 | 0.129 | yes |
| 10 | Y | +2 | +2 | 2.8 | 12.22 | 0.132 | yes |
| 11 (extra) | Y | 0 | +2 | 2.0 | 12.28 | 0.132 | |
| 12 (extra) | Y | 0 | +3 | 3.0 | 12.41 | 0.134 | yes |
| 13 (extra) | Y | +2 | +3 | 3.6 | 11.88 | 0.134 | |
| 14 (extra) | Y | −3 | +4 | 5.0 | 12.03 | 0.239 | |
| 15 (extra) | Y | 0 | +3 | 3.0 | 12.03 | 0.139 | yes |

10/10 counted cycles successful, and 15/15 including the extras. In the counted cycles:
- place error 1.4–8.0 mm (mean 3.9 mm);
- cycle time 11.88–12.44 s (mean 12.13 s).

**Result (R13, R1, R3): PASS.** Every cycle with both payloads was successful, inside 15 s and inside 10 mm.

### 4.4 Effect of the oscillation before the drop

The oscillation was seen in 2 of 10 cycles at 50 g and in 9 of 15 at 500 g. Place error by group (all 25 cycles):

| Group | n | Mean error (mm) | Max error (mm) | Mean radial (mm) | Mean lateral (mm) |
|---|---|---|---|---|---|
| 50 g, no oscillation seen | 8 | 2.3 | 3.6 | +1.8 | −1.3 |
| 50 g, oscillation seen | 2 | 4.7 | 5.0 | +4.0 | −2.5 |
| 500 g, no oscillation seen | 6 | 3.1 | 5.0 | 0.0 | +2.8 |
| 500 g, oscillation seen | 9 | 4.1 | 8.0 | +0.2 | +3.3 |

- **No significant effect on placement.** With 500 g, the difference between cycles with and without the oscillation is not significant (Mann-Whitney U, p = 0.38), and every cycle stayed within the 10 mm tolerance.
- **The oscillation is present in every cycle.** The GUI data log (`csv/8.7_descent_analysis.csv`) shows the commanded descent overshoots each time:
  - the model tip dips to 5–16 mm above the table (9–20 mm below the 25 mm place height) and rebounds;
  - the commanded shoulder swings 4–8°;
  - the release comes 0.6–1.0 s after the descent starts, so the cube is released while the arm is still settling.

  Whether the oscillation is visible depends on where in the swing the release falls.
- **The lateral error depends on payload and on the base.** It differs significantly between payloads: 50 g lands 1.5 mm left of the ray on average, and 500 g lands 3.1–3.2 mm right of it (p < 0.001). It follows the base encoder angle at the moment of release (Spearman ρ = 0.76), so it is set by where the base settles with its trim off, not by the oscillation.
- **Two cycles drew a higher holding current:** 0.317 A (500 g cycle 8) and 0.239 A (cycle 14), against about 0.13 A normally. Both were still placed within 3 mm and 5 mm.

## 5. Test 8.8: power, temperature and E-stop (R2, R4, R5, R6, R7)

### 5.1 Full-extension hold with 500 g (R2)

- **Method:** `W,0,0,0,0,10` moved all joints to 0°, with every link horizontal and the arm straight forward (tip about 450 mm from the base axis). The steps were:
  1. Unloaded: measure the height of the gripper pad centre above the table with a rule, and read the joint angles with `P`.
  2. Place the 500 g cube in the jaws and send `G,1` (grasp confirmed, median 0.142 A).
  3. Measure the pad height again and read `P`.
  4. Hold for 5 min.
- **Pass criterion:** the arm holds the load at full extension.

| Quantity | Empty | 500 g | Change |
|---|---|---|---|
| Pad-centre height above table | 65 mm | 50 mm | **−15 mm** |
| Shoulder encoder | +0.13° | −0.62° | −0.75° |
| Elbow encoder | −0.25° | −0.88° | −0.63° |
| Wrist encoder | −0.09° | −0.18° | −0.09° |
| Shoulder trim | +2.33° | +5.00° (clamp) | |
| Elbow trim | +2.39° | +5.00° (clamp) | |
| Wrist trim | −0.23° | +1.72° | |
| Gripper holding current | | 0.109 A median | |

**Result: PASS.** The arm held 500 g at full extension for 5 min.

- **Sources of the sag:** the measured joint deflections account for about 9 mm of the 15 mm (0.75° × 453 mm + 0.63° × 278 mm + 0.09° × 133 mm). The rest is bending and play beyond the encoders: the wrist bracket, the gripper jaws and the cube in the pads.
- **The trims are at their limit.** The shoulder and elbow trims reached their +5° clamp, so this pose with 500 g is the limit of what the trim loop can correct.

### 5.2 Rail voltages (R5)

- **Method:** multimeter on each buck-converter output (and the 5 V logic supply) in three conditions:
  1. idle at home;
  2. during the 500 g full-extension hold;
  3. over one 8.7 cycle with 500 g.

  For the cycle, the meter display was filmed together with the arm, and the lowest and highest readings were taken from the video frame by frame. The meter updates about 3 times a second, so short transients are not captured.
- **Pass criterion:** every reading within the rated voltage range of the servos on that rail.

| Rail | Rating (V) | Idle (V) | Holding 500 g extended (V) | Cycle min (V) | Cycle max (V) | Result |
|---|---|---|---|---|---|---|
| #1 shoulder (DS5180), 8.0 V | 6.0–8.4 | 8.12 | 8.05 | 7.78 (side transfer) | 8.08 (lowering) | PASS |
| #2 base + elbow (DS3240), 6.7 V | 4.8–6.8 | 6.67 | 6.63 | 6.65 | 6.66 | PASS |
| #3 wrist + gripper (DS3225, MG996R), 6.5 V | 4.8–6.8 | 6.54 | 6.50 | 6.51 (side transfer) | 6.53 (descents, pickup) | PASS |
| 5 V logic | 4.75–5.25 | 5.15 | 5.06 | 5.15 | 5.15 | PASS |

The ratings are typical published ranges; check each against the servo's own datasheet before quoting.

**Result: PASS.**
- The largest dip was 0.34 V below idle, on the shoulder rail during the side transfer.
- No rail rose above its idle value.
- Rail 2 sits 0.13 V below the DS3240's 6.8 V maximum, the smallest margin in the system.
- On rail 2 the elbow loads the rail on the descents and the base on the side transfer. Their peaks do not coincide, so that rail stayed within 0.02 V.

### 5.3 Temperatures (R4)

- **Method:** Fluke VT02 readings of each servo case, each buck converter, the bench supply and the E-stop relay, taken at three points:
  1. baseline after about 5 min holding home, before testing;
  2. about 5 min after the last 8.7 cycle (25 cycles in total, 15 of them with 500 g);
  3. straight after the 5 min full-extension hold with 500 g.

  The ESP32 and INA219 were not imaged, because they sit inside the wiring loom.
- **Pass criterion:** every servo case ≤ 60 °C. This is an assumed safe temperature for the PLA-CF mounts and the servos, not a datasheet value.

| Component | Baseline (°C) | After 8.7 (°C) | After 5 min extended hold, 500 g (°C) |
|---|---|---|---|
| Room | 19.9 | 19.0 | 19.0 |
| Base servo (DS3240) | 21.3 | 19.5 | 21.3 |
| Shoulder servo (DS5180) | 23.3 | 23.9 | 32.1 |
| Elbow servo (DS3240) | 27.7 | 29.8 | **54.5** |
| Wrist servo (DS3225) | 20.9 | 19.4 | 23.4 |
| Gripper servo (MG996R) | 21.0 | 19.7 | 21.7 |
| Buck #1 (shoulder) | 27.3 | 28.0 | 29.1 |
| Buck #2 (base + elbow) | 25.2 | 26.2 | 29.8 |
| Buck #3 (wrist + gripper) | 22.6 | 27.1 | 24.8 |
| Bench power supply | 30.3 | 30.2 | 31.3 |
| E-stop relay | not measured | 42.8 | 41.7 |

**Result: PASS.** The hottest servo was the elbow at 54.5 °C (+35.5 °C over room) after the extended hold, against the assumed 60 °C limit.

- **Normal cycling:** after 25 cycles the servos stayed within 10.8 °C of room temperature.
- **The extended hold heats the elbow most.** It carries the forearm, wrist, gripper and 500 g at full reach, with its trim at the clamp.
- **The relay is the hottest part in normal use.** It was the warmest component after 8.7 and nearly as warm after the hold. Its coil is energised the whole time the arm runs, and the servo current passes through its contacts.

### 5.4 E-stop (R7)

- **Method:** with a soft surface under the arm, the E-stop was pressed 10 times during a slow move (`W,0,40,-60,-70,15`). For each press the following were checked:
  - the servo rail voltage (multimeter);
  - whether the 5 V logic and the ESP stayed on;
  - whether the GUI showed the E-stop;
  - whether the arm stayed still until the latch was released *and* Reset was pressed in the GUI;
  - the stop time, from iPhone video.

  One more press was made with the E-stop signal wire to GPIO 41 disconnected.
- **Pass criterion:** the servo rails drop to < 0.5 V on every press, the logic stays on, and the arm does not restart until it is reset.

| Press | Servo rails | 5 V logic / ESP on | GUI shows E-stop | No restart until reset | Result |
|---|---|---|---|---|---|
| 1–10 | 0 V on every rail, every press | Y | Y | Y | PASS |
| GPIO 41 wire disconnected | 0 V | Y | N (expected: the ESP can't see the button) | N | PASS (power cut) |

The stop time over the 10 presses was 0.20–0.34 s.

**Result: PASS.**
- **Normal presses:** every press cut all servo power through the relay, the ESP and logic stayed powered, and the arm restarted only after the latch was released and Reset was pressed in the GUI.
- **Signal wire disconnected:** the relay still cut the servo rails, so the power cut works in hardware without the microcontroller. But when the E-stop was released, the rails came back without a reset and the arm jerked to its last commanded pose. The restart interlock depends on the GPIO 41 signal.

### 5.5 Fuse (R6)

The fuse was intact after all tests. Polarity protection and transient suppression were checked by inspection only. **Result: PASS.**

## 6. Points for the write-up

- **8.6:**
  - accuracy (3.1–6.4 mm) and repeatability (4.7–7.0 mm) are both well inside ±10 mm;
  - the error is mostly a radial shortfall that is the same sign at every target (geometric), plus a growth over repeated approaches that the joint angles don't explain;
  - the tip moves about 5 mm per degree of joint error, so sub-degree joint control is what the requirement needs.
- **8.7:**
  - cycle times sit at about 12 s, a 3 s margin on 15 s;
  - the oscillation before the release is a command overshoot on the fast descent and has no significant effect on placement;
  - a pause between reaching the place height and releasing would remove it;
  - the lateral place error follows where the base settles (base trim off, play at the horn).
- **8.8:**
  - the 500 g full-extension hold saturates the shoulder and elbow trims and heats the elbow to 54.5 °C, so this is the design's load limit, not a working condition;
  - the rails are stiff (≤ 0.34 V dip);
  - rail 2's margin to the DS3240 maximum is small (0.13 V);
  - the E-stop's power cut is hardware-only, but the no-restart behaviour relies on the signal wire. A latching relay or a firmware fault on an open signal line would remove that dependency;
  - the relay runs at about 42 °C.
- **Limitations to state:**
  - offsets were read by eye to 1 mm;
  - the meter catches only slow changes (about 3 readings a second);
  - the 60 °C temperature limit is an assumption;
  - the rail ratings must be confirmed against the datasheets;
  - T10 and T11 (8.6 with 500 g) were not run.

## 7. Files

| File | Contents |
|---|---|
| `WDD2_Test_Log.xlsx` | All test sheets with formulas: Overview, 8.2 to 8.8 (days 1 and 2) |
| `csv/8.6_approaches.csv` | Every 8.6 approach: target, radial and lateral offset, error to target |
| `csv/8.6_summary.csv` | Per target: centroid, accuracy, mean and std of l, repeatability, worst single approach |
| `csv/8.6_joint_commands.csv` | Base angle, via-point and target joint angles used for each target |
| `csv/8.7_cycles.csv` | All 25 cycles: payload, counted or extra, offsets, place error, cycle time, holding current, oscillation seen, GUI cycle number and export file |
| `csv/8.7_descent_analysis.csv` | Per cycle, from the GUI data log: descent-to-release time, model tip overshoot, commanded and measured joint swing, base command and encoder at release |
| `csv/8.8_full_extension_hold.csv` | Pad heights, joint angles, trims and gripper current, empty and with 500 g |
| `csv/8.8_rails.csv` | Rail voltages: idle, holding 500 g extended, minimum and maximum over one 8.7 cycle |
| `csv/8.8_thermal.csv` | Case temperatures at the three measurement points |
| `csv/8.8_estop.csv` | Each E-stop press and the signal-wire-disconnected run |
| `raw/gui_export_8.7_50g_*.xlsx` | GUI export for the 50 g set: summary, cycle reports, 50 Hz data log. Counted cycles are GUI cycles 3–8 and 10–13; cycle 2 was the trial run, and cycles 1 and 9 were stopped before completion. |
| `raw/gui_export_8.7_500g_*.xlsx` | GUI export for the 500 g set. Cycles 1, 3–9 and 11–17 are the 15 completed cycles in order; cycles 2 and 10 were stopped before completion. The data log in this file also contains the 50 g set (t < 1310 s). |

The cube mass setting in the GUI was 0.5 kg for both exports. It affects only the model current columns, not any measured value.
