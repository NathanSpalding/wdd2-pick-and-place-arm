# WDD2 testing, day 1 (12 October 2026)

Chapter 8 test results for the WDD2 4-DOF pick-and-place arm (Mechatronic Project 478). This file is the starting point for the write-up: what was tested, how, the numbers, what they mean, and what is still open. All raw data is in the `raw_logs/` and `csv/` folders next to this file, and the worked sheets are in `WDD2_Test_Log.xlsx`.

## 1. Summary

| Req. | What | Test | Result | Key number |
|---|---|---|---|---|
| R8 | Encoder accuracy ±1° | 8.2 | **PASS** | Max error 0.37–0.61° per joint, against an independent reference |
| R9 | Encoder finer and more accurate than the servo | 8.3 | **PASS** | Encoder noise ≤ 1 count (0.088°); servo steps 0.27–0.78°; servo reversal deadband 0.64–3.6° |
| R11 | Control loop ≥ 200 Hz | 8.4 | **PASS** | Mean period 5000.0 µs, 0 overruns; the trim loop cuts steady-state error 5–25× on the shoulder, elbow and wrist |
| R10 | Grasp detection 9/10 | 8.5 | **PASS** | Detection correct 70/70; detection and hold through a lift ≥ 9/10 at every payload up to 500 g |
| R12 | Positioning ±10 mm | 8.6 | not done | Target mat ready |
| R13, R1, R3 | Cycles, reach, payload | 8.7 | not done | |
| R2, R4–R7 | Power, temperature, E-stop | 8.8 | not done | |

## 2. Test set-up common to all tests

- **Firmware:** `WDD2_firmware.ino` on the ESP32-S3 over native USB (COM6, 115200 baud, USB CDC On Boot enabled). The 200 Hz loop reads 4 × AS5600 through a TCA9548A and one INA219 at 400 kHz I²C. Commands are typed in the Arduino Serial Monitor (GUI closed).
- **Bring-up sketch:** `WDD2_bringup.ino` for the raw-pulse tests in 8.3 part 2 (`p<n> <us>` sets a pulse, `m<n>` monitors one encoder).
- **Joint angle:** encoder angle = DIR × wrap(raw − ZERO_RAW) × 360/4096.
  - ZERO_RAW = {1055, 117, 645, 444}
  - DIR = {+1, −1, −1, −1}
  - base, shoulder, elbow, wrist
- **Servo fits from bring-up** (pulse = P0 + angle/K):

| Joint | P0 (µs) | K (°/µs) | Fit RMS (°) | Fit max (°) | Hysteresis (°) |
|---|---|---|---|---|---|
| Base | 1623 | +0.1276 | 0.45 | 0.74 | 0.5 |
| Shoulder | 1558 | −0.1292 | 1.60 | 2.96 | 3.2 |
| Elbow | 1562 | −0.1358 | 0.41 | 0.90 | 1.3 |
| Wrist | 1548 | −0.1361 | 1.26 | 3.51 | 0.7 |

- **Trim loop (outer integral loop on encoder error, only while the joint is still):**
  - KI = {1, 2, 2, 2} °/s per degree of error
  - deadband = {0.5, 0.3, 0.3, 0.3}°
  - clamp = {3, 5, 5, 5}°
  - settle time before trimming 150 ms
- **Motion:** S-curve trajectory (J stream extrapolated ≤ 40 ms, rate-limited, then two 80 ms moving averages in series). Test moves use `M,joint,angle,speed`.
- **Measurement:** `O,joint,seconds` prints the mean, standard deviation and peak-to-peak of the encoder angle over a window. `V,1` turns on 50 Hz telemetry (`A,b,s,e,w` angles, `I,amps` gripper current).

## 3. Test 8.4 part 1: control loop rate (R11)

**Method:** `T` resets the timing counters; after about 114 s of running (including moves) `T` prints the loop statistics. The firmware times every loop with `micros()`.

| Quantity | Value |
|---|---|
| Loops measured | 22 773 |
| Mean loop period | 5000.0 µs (200.0 Hz) |
| Min / max loop period | 4717 / 5283 µs |
| I²C read, mean / max (4 encoders + INA219) | 1472 / 1675 µs |
| Loop execution, max | 1965 µs |
| Overruns | 0 |

**Result: PASS.** Every loop finished in under 2 ms of its 5 ms slot (39 % worst-case load). The ±283 µs jitter in the period is the scheduler waking early or late around the 5 ms tick; the mean is exact and no tick was missed.

## 4. Test 8.3: encoder resolution and servo deadband (R9)

### 4.1 Part 1: encoder noise at a fixed pose (60 s hold)

`M,joint,angle,40`, wait 3 s, `O,joint,60`.

| Joint | Pose | Std | Peak-to-peak |
|---|---|---|---|
| Base | 30° | 0.00° | 0.00° (0.09° in a first run) |
| Shoulder | 30° | 0.00° | 0.00° (0.09° in a first run) |
| Elbow | 0° | 0.00° | 0.00° |
| Wrist | 0° | 0.00° | 0.00° |

One encoder count is 360/4096 = 0.088°. No joint moved by more than 1 count in 60 s.

The first base/shoulder run printed a wrong std (0.33°) because of floating-point cancellation in the variance sum. The firmware now uses Welford's running variance; the re-run is the value above.

Held means were 29.62° (base) and 29.79° (shoulder) for a 30° target, inside the trim deadbands of 0.5° and 0.3°.

### 4.2 Part 2: servo deadband (bring-up sketch, 1 µs steps)

**Method:**
1. Approach the start pulse from 20 µs below.
2. Step the pulse **down** 1 µs at a time, about 3 s per step, monitoring the raw encoder count. The first pulse where the count changes and stays changed is the **reversal deadband** (servo deadband plus gear and horn backlash).
3. Keep stepping and note the spacing of the following jumps. That spacing is the **step resolution**.

The shoulder and elbow were at 0°.

| Joint | Start | Reversal: first move | Reversal deadband | Step resolution | Behaviour |
|---|---|---|---|---|---|
| Base | 1623 µs | 1618 µs | 5 µs = **0.64°** | ~4 µs = **0.51°** | Hunts ±0.7° for several seconds after every step (horn play) |
| Shoulder | 1558 µs | 1530 µs | 28 µs = **3.62°** | 6 µs = **0.78°** | Stick-slip: holds, then slips 0.5–0.8° in one jump, sometimes about 2 s after the command |
| Elbow | 1582 µs | 1576 µs | 6 µs = **0.81°** | 2 µs = **0.27°** | Cleanest joint |
| Wrist | 1548 µs | 1539 µs | 9 µs = **1.22°** | 3–4 µs = **0.48°** | Small jumps; a 1-count creep at 1541 µs |

Degrees use the fitted slope |K|. The shoulder's 3.6° reversal agrees with its 3.2° hysteresis from the bring-up fit. On average the shoulder still follows the fit (6 µs × 0.129 = 0.78° predicted, 0.79° measured); it just moves in lumps.

### 4.3 Part 3: comparison

| Joint | Encoder noise | Servo step resolution | Encoder max error (8.2) | Servo open-loop max error (fit) | Result |
|---|---|---|---|---|---|
| Base | ≤ 0.09° | 0.51° | 0.61° | 0.74° | PASS |
| Shoulder | ≤ 0.09° | 0.78° | 0.37° | 2.96° | PASS |
| Elbow | 0.00° | 0.27° | 0.57° | 0.90° | PASS |
| Wrist | 0.00° | 0.48° | 0.37° | 3.51° | PASS |

The encoder resolves 3–9× finer than the smallest servo movement, and is more accurate than the servo open loop on every joint. That is the case for closing the loop on the encoders.

## 5. Test 8.2: encoder calibration against an independent reference (R8)

**Reference:**
- **Shoulder, elbow and wrist:** iPhone Measure → Level, zeroed on the table, phone on its long edge against the same face of each link (about ±0.5°). The level shows magnitude only, so below-horizontal readings were given a negative sign.
  - **Elbow:** angle = forearm − upper arm (shoulder held at 30°).
  - **Wrist:** angle = gripper − forearm (forearm 30.0°, the mean of a start and end reading of 30.2° and 29.8°).
- **Base:** protractor centred on the base axis, 0° on a fixed edge of the base plate, a pointer on the base link, read from above (about ±0.5°).

**Method:** one upward sweep per joint in 10° steps. At each pose: `M,j,angle,40`, wait 3 s, `O,j,2` (2 s mean of the encoder), then the reference reading.

| Joint | Range | Points | Max abs. error | RMS error | Mean offset |
|---|---|---|---|---|---|
| Base | −60 to +60° | 13 | 0.61° | 0.28° | −0.03° |
| Shoulder | −30 to +75° | 12 | 0.37° | 0.23° | +0.03° |
| Elbow | −90 to +90° | 19 | 0.57° | 0.32° | +0.21° |
| Wrist | −80 to +90° | 18 | 0.37° | 0.17° | +0.04° |

**Result: PASS on every joint.** Error = encoder − reference.
- **No ZERO_RAW change was needed.** The largest mean offset (elbow, +0.21°) is about 2 counts, smaller than the uncertainty of the two-reading elbow reference.
- **The errors are at the level of the reference uncertainty.** The honest statement is: "the encoders agree with an independent ±0.5° reference within its uncertainty, max 0.61°".
- **Base, −30° to +30°:** the base hunted and was touched to make it settle before reading. That only stops the swing; the encoder and protractor then read the same settled position. These points are marked in the CSV.
- **Elbow fault:** during the shoulder sweep the elbow encoder stopped answering (`F,elbow,ENC`, more than 20 missed reads in a row) as the shoulder moved to 0°. `R` re-armed it and it did not recur. The likely cause is the elbow encoder cable being pulled across the shoulder joint. Note it for the cable-routing section.

## 6. Test 8.4 part 2: step response with the trim loop on and off (R11, Objective 6)

**Method:**
1. Pose: shoulder 30°, elbow 0°, wrist 0° (the shoulder blocks started with the elbow and wrist at 0°).
2. `V,1` (telemetry on). Each block is 3 steps of +10° and back at 180 °/s with the trim on (`X,j,1`), then 3 with it off (`X,j,0`). Waits were about 5 s.
3. The 500 g blocks held the cube stack in the gripper (`G,1`, `K,1` confirmed).
4. Metrics from the 50 Hz `A` lines:
   - **final error** = mean of 4.5–5.0 s after the command, minus the target;
   - **settling** = the last time outside ±0.5° of that final value;
   - **overshoot** = the furthest point past the target.

**Steps** were 20° → 30° (base 35° → 45°, chosen outside the ±30° hunting zone).

| Joint | Load | Mean abs. final error, trim off | Mean abs. final error, trim on | Improvement | Trim-on settling | Trim-on overshoot |
|---|---|---|---|---|---|---|
| Shoulder | 0 g | 2.46° | 0.50° | 1.96° | 2.6–3.1 s | 1.2–1.3° |
| Shoulder | ~500 g | 5.65° | 0.44° | 5.21° | 0.5–0.6 s | 0° |
| Elbow | 0 g | 3.61° | 0.20° | 3.41° | 0.3–0.4 s | 0.2–0.5° |
| Elbow | ~500 g | 4.34° | 0.18° | 4.16° | 0.5–1.7 s | 1.0–1.4° |
| Wrist | 0 g | 1.40° | 0.19° | 1.21° | 0.2–2.8 s | 0.15–0.4° |
| Wrist | ~500 g | 1.73° | 0.17° | 1.56° | 0.3–0.5 s | 0.4–1.3° |
| Base | 0 g | 0.15° | 0.12° | 0.02° | 0.3 s | 0.8–1.0° |

**What the data shows:**
1. **Open loop, every arm joint stops short and sags under gravity.**
   - Shoulder: 2.5° short at 0 g, 5.7° at 500 g (it reaches only 24.4° for a 30° command).
   - Elbow: about 3.6° low at 0 g, 4.3° at 500 g.
   - These open-loop errors are pose- and load-dependent, and larger than the bring-up fits suggest, because the fits were taken at a different pose with no payload.
2. **With the trim on, every arm joint finishes within 0.5°**, loaded or not. That is a 5–25× reduction in steady-state error.
3. **The cost is settling time on the shoulder.** At 0 g the trim value learnt at the previous position carries over, giving about 1.3° overshoot. The shoulder then has to reverse through its 3.6° backlash in 0.5–0.8° slips, which takes 2.5–3 s. With 500 g the load takes up the backlash and the overshoot disappears.
4. **Stick-slip limit cycle on the shoulder.** On one trim-on return step the shoulder held at 20.6° for about 4 s, then slipped to 19.0°. Its slips (about 0.8°) are bigger than the 0.3° trim deadband, so it can hop around the target by about 1°.
5. **Joint coupling.**
   - Each elbow step with 500 g pushed the shoulder about 1.3° off target, and the shoulder's own trim brought it back within about 5 s.
   - Each wrist step nudged the elbow by up to 0.6°, with the same recovery.
6. **Base: the trim loop caused hunting.** With the trim on, 3 of 6 holds swung ±0.44° continuously (about 0.56 s period), and the shoulder wobbled in step. With it off, every hold was still (peak-to-peak ≤ 0.03°) and within 0.3° of target. The base is accurate open loop, and the integral action re-excites the horn play every time the swing crosses the 0.5° deadband. **The base trim is now off by default in the firmware** (`TRIM_ON = {false, true, true, true}`; `X,0,1` switches it back on).
7. **Gripper grip relaxes.** During the 500 g elbow block the gripper current fell from about 0.12 A to about 0.075 A over a few minutes. That is still well above the 0.04 A threshold, and no grip loss was reported.

## 7. Test 8.5: grasp detection (R10)

**Detection rule (firmware):** 450 ms after `G,1`, take the median of 40 current samples over 200 ms. If it is above 0.04 A, report `K,1` (grasp); otherwise report `K,0` and re-open the gripper. `K,2` means grip lost while holding.

**Method:**
1. Pose: base 40°, shoulder 30°, elbow 0°, wrist 0°.
2. Hold the payload in the open jaws, `G,1`, and let go after `K,1`.
3. Lift 20° with the shoulder (`M,1,50,40`), hold 3 s, lower (`M,1,30,40`), `G,0`.
4. A trial counts as correct if the K line is right **and** the payload is held through the lift without slipping.

Empty closures are correct if `K,0`. Payloads are the 24 g cube with weights taped to it. The masses below are the sums of the weighed items; weigh each assembled stack, tape included, before quoting final masses.

| Payload | Detection correct | Detection and hold correct | Result | Median current (mean, range) |
|---|---|---|---|---|
| Empty | 10/10 | 10/10 | PASS | 0.009 A (all) |
| 50 g (52 g) | 10/10 | 10/10 | PASS | 0.122 A (0.102–0.145) |
| 100 g (94 g) | 10/10 | 10/10 | PASS | 0.117 A (0.105–0.140) |
| 200 g (197 g) | 10/10 | 10/10 | PASS | 0.116 A (0.108–0.138) |
| 300 g (306 g) | 10/10 | 10/10 | PASS | 0.119 A (0.099–0.145) |
| 400 g (399 g) | 10/10 | 9/10 | PASS | 0.137 A (0.120–0.148) |
| 500 g (498 g) | 10/10 | 9/10 | PASS | 0.158 A (0.133–0.212) |

The 100 g set also had an 11th trial, which held (0.108 A).

**What the data shows:**
1. **Detection was right in all 70 trials.** The gap between empty (0.009 A) and the lowest grasp (0.099 A) is more than 10×, so the 0.04 A threshold has a wide margin on both sides.
2. **Current does not measure mass.** From 50 to 300 g the median stays at about 0.12 A, because the jaws close to the same pulse (950 µs) on the same cube, so the servo stalls against the same width. Only at 400–500 g does it rise (to 0.158 A mean), as the load pulls on the jaws and the servo works to hold position.
3. **The weak point is the hold, not the detection.** The two failures (one each at 400 g and 500 g) were the payload rotating about the jaw axis during the lift. The weights hang off-centre from the grip line, and a two-jaw pinch resists rotation poorly. Neither dropped, and neither tripped `K,2`, because the cube stayed in the jaws. The heaviest payloads pass with no margin (exactly 9/10). Proposed improvement: rubber or grip-tape jaw pads, or a payload with its mass centred under the jaws.
4. The slipping trials had normal currents (0.120 A and 0.133 A), so the current cannot predict a slip.

## 8. Firmware changes made today

1. **Welford running variance** in the `O` command (fixes the wrong std from float cancellation).
2. **Telemetry off by default** (`V,1` to turn on). The start-up banner is printed when the USB host connects, debounced to 1 s.
3. **Base trim loop off by default** (from the 8.4 step test, see section 6, point 6).

## 9. Problems met and how they were handled

| Problem | Cause | Fix |
|---|---|---|
| Serial Monitor silent after uploading | ESP32-S3 native USB re-enumerates on reset; the monitor stays on the old connection | Close and reopen the monitor, or replug USB |
| Std of 0.33° with peak-to-peak 0.09° | Float cancellation in the variance sum | Welford's algorithm in firmware |
| `F,elbow,ENC` during the shoulder sweep | Elbow encoder cable pulled or flexed across the shoulder | `R` re-armed it; give the cable a slack loop |
| Base hunting near home and between ±30° | Gear/horn play, kept going by the base trim integral | 25T aluminium horn (earlier), base trim off (today); test targets placed outside ±30° |
| Shoulder moves in 0.5–0.8° jumps | Stick-slip in the gear train under gravity load | Trim loop compensates; report as a limitation |
| Payload rotation at 400–500 g | Off-centre load about a two-jaw pinch | Proposed: grip pads or centred payload |

## 10. Points for the write-up

- **R9 argument:**
  - The encoder resolves 0.088°.
  - The servo cannot move in steps smaller than 0.27–0.78°, and after a reversal it can sit up to 3.6° off target.
  - Open-loop errors grow with pose and load, up to 5.7° at the shoulder with 500 g. Only the encoders can see this, and the trim loop removes it.
- **Separate detection from holding** in 8.5. The sensing works perfectly (70/70); the limit is mechanical.
- **Explain why the base is open loop:** the integral action plus backlash made a limit cycle, and the open-loop base is already within 0.3°. This is a good "tested a design decision and changed it" story.
- **Measurement uncertainty:**
  - 8.2 reference about ±0.5°.
  - Encoder 1 count = 0.088°.
  - Step metrics from 50 Hz telemetry (20 ms resolution in settling time).
- **Payload masses:** quote the weighed totals (the item sums differ from the first-listed totals by 1–3 g).
- **Graphs to make from the CSVs:**
  - 8.2: encoder vs reference for each joint (error vs angle).
  - 8.3: raw count vs pulse for the shoulder reversal, showing the stick-slip staircase.
  - 8.4: angle vs time for one step with the trim on and off (shoulder 500 g and elbow 0 g are the clearest), and the base hunting with the trim on vs still with it off.
  - 8.5: median current vs payload mass, with the empty-closure band and the 0.04 A threshold.

## 11. Still to do

- **8.6 accuracy and repeatability (R12):**
  - Print `WDD2_target_mat_A4x4.pdf` at 100 %.
  - Fit a pointer at the TCP.
  - Targets T1–T9 at 0 g, T10/T11 with 500 g, 10 approaches each.
  - First check that `M,0,35,40` points at the +35° ray.
- **8.7 pick-and-place cycles (R13, R1 reach ≥ 300 mm, R3 500 g payload).**
- **8.8 power, temperatures and E-stop (R2, R4–R7).** The rail ratings come from the datasheets; the 60 °C temperature limit is an assumption, to state as such.
- Optional: extra 400 g grasp trials (to report /20), and a "with grip pads" set at 400 and 500 g.

## 12. Files

| File | Contents |
|---|---|
| `WDD2_Test_Log.xlsx` | All test sheets with formulas: Overview, 8.2 to 8.8 |
| `csv/8.2_encoder_vs_reference_sweep1.csv` | Every 8.2 point: commanded, gauge readings, reference, encoder, error |
| `csv/8.3_encoder_noise_60s.csv` | 60 s hold std and peak-to-peak per joint |
| `csv/8.3_servo_deadband.csv` | Reversal deadband and step resolution per joint |
| `csv/8.4_loop_timing.csv` | Loop timing from `T` |
| `csv/8.4_step_metrics_all_steps.csv` | Every step (up and return), trim on/off, settling, overshoot or sag, final error |
| `csv/8.4_timeseries_<joint>_<load>.csv` | 50 Hz angles of all 4 joints, gripper current and command events (t in s from the start of each log) |
| `csv/8.5_grasp_trials.csv` | All 70 grasp trials with K result, hold, median current |
| `csv/8.5_payloads.csv` | Payload make-up and item masses |
| `raw_logs/8.3_deadband_*.txt` | Bring-up sketch monitor logs for the deadband test |
| `raw_logs/8.4_step_*.txt` | Serial Monitor logs of the step tests |

Notes on the raw logs:
- `8.4_step_elbow_0g.txt` also contains the shoulder 500 g run at the start, because the monitor wasn't cleared in between. The elbow block starts at the line `# test move: elbow to 20.0 deg at 40 deg/s`.
- `8.3_deadband_wrist_and_base.txt` contains the wrist run followed by the base run.
- The 8.2 and 8.5 data were typed into the chat rather than logged to a file; they are complete in the CSVs and the spreadsheet.
