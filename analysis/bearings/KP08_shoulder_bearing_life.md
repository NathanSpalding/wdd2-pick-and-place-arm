# KP08 Shoulder Bearing – Rolling-Contact Bearing Life Analysis

**Project:** WDD2 pick-and-place robotic arm (Mechatronic Project 478)
**Component:** KP08 pillow-block insert bearing at the shoulder joint (idler side)
**Method:** Budynas & Nisbett, *Shigley's Mechanical Engineering Design*, 10th ed., Chapter 11

---

## 1. Assumptions

| Item | Value | Basis |
|---|---|---|
| Bearing type | Single-row deep-groove ball insert, 8 mm bore | KP08 (KP8UU insert, SAE 52100 steel) |
| Basic dynamic load rating, C₁₀ | **3.30 kN** (742 lbf) | Supplier data; the lower of two published values (3.30 / 4.2 kN) is used |
| Basic static load rating, C₀ | **1.30 kN** (292 lbf) | Supplier data; lower published value (1.30 / 1.6 kN) |
| Rating life, L_R | 10⁶ rev at 90 % reliability | Standard catalogue basis |
| Load-life exponent, a | 3 | Ball bearing, Eq. (11–1) |
| Radial load, F_r | **6.6 N** | Share of the arm load carried by the shoulder idler bearing (other share carried by the DS5180 output) |
| Axial load, F_a | 0 N | Shoulder shaft is horizontal; no thrust |
| Rotation factor, V | 1 | Inner ring rotates with the shaft; housing is stationary |
| Load-application factor, a_f | **1.2** | Table 11–5: machinery with light impact (servo start/stop), 1.2–1.5 |
| Weibull parameters | x₀ = 0.02, (θ − x₀) = 4.439, b = 1.483 | Shigley typical ball-bearing values (p. 601, Ex. 11–3) |
| Design reliability, R_D | 0.99 | Single critical support bearing |
| Duty cycle | 0.5 rev of total travel per 15 s cycle → **n ≈ 2 rev/min** average | Shoulder lifts and lowers twice per pick-and-place cycle |

---

## 2. Equivalent Radial Load — Eq. (11–12), Table 11–1

F_a = 0, so F_a /(V F_r) = 0 ≤ e and the first set of factors applies: X₁ = 1, Y₁ = 0.

F_e = X₁ V F_r + Y₁ F_a = 1(1)(6.6) + 0 = **6.6 N**

Design load including the application factor:

F_D = a_f F_e = 1.2 × 6.6 = **7.92 N**

---

## 3. Rating Life (90 % reliability) — Eq. (11–3)

L₁₀ = (C₁₀ / F_D)ᵃ × L_R = (3300 / 7.92)³ × 10⁶ = (417)³ × 10⁶

**L₁₀ ≈ 7.2 × 10¹³ rev**

At n = 2 rev/min: l₁₀ = L₁₀ / (60 n) ≈ **6.0 × 10¹¹ h**

---

## 4. Life at 99 % Reliability — Eq. (11–9) rearranged

x_D = (C₁₀ / F_D)ᵃ · [x₀ + (θ − x₀)(ln 1/R_D)^(1/b)]

[x₀ + (θ − x₀)(ln 1/0.99)^(1/1.483)] = 0.02 + 4.439(0.01005)^0.674 = **0.220**

L₉₉ = 7.23 × 10¹³ × 0.220 ≈ **1.6 × 10¹³ rev**  →  l₉₉ ≈ **1.3 × 10¹¹ h**

---

## 5. Check Against Required Life — Eq. (11–10), Table 11–4

Requirement: Table 11–4, *machines for intermittent service where reliable operation is of great importance*, **8 kh** at R = 0.99.

L_D = 8000 × 60 × 2 = 9.6 × 10⁵ rev  →  x_D = 0.96

C₁₀,req = a_f F_e [x_D / (x₀ + (θ − x₀)(1 − R_D)^(1/b))]^(1/3) = 7.92 × (0.96 / 0.220)^(1/3) = **13 N**

C₁₀,available / C₁₀,req = 3300 / 13 ≈ **255**

---

## 6. Static Load Check

Pure radial load, so P₀ = F_r:

s₀ = C₀ / P₀ = 1300 / 6.6 ≈ **197**  (typical minimum 1–2 for smooth operation)

---

## 7. Sensitivity

| Case | F_r | V | F_D = a_f V F_r | L₉₉ (rev) | l₉₉ at 2 rev/min | C₁₀,req (8 kh, R = 0.99) | s₀ |
|---|---|---|---|---|---|---|---|
| Baseline | 6.6 N | 1.0 | 7.9 N | 1.6 × 10¹³ | 1.3 × 10¹¹ h | 13 N | 197 |
| Outer ring rotates | 6.6 N | 1.2 | 9.5 N | 9.2 × 10¹² | 7.7 × 10¹⁰ h | 16 N | 197 |
| Shoulder idler share with ×1.5 design factor (16.5 N / 2) | 8.25 N | 1.0 | 9.9 N | 8.1 × 10¹² | 6.8 × 10¹⁰ h | 16 N | 158 |
| Entire arm load on this bearing | 16.5 N | 1.2 | 23.8 N | 5.9 × 10¹¹ | 4.9 × 10⁹ h | 39 N | 79 |

Even the most severe case, where the full design load sits on this one bearing with outer-ring rotation, still needs only 39 N of the 3300 N dynamic rating.

**Oscillating motion:** the shoulder moves through moderate arcs (typically 20–60°), and the bearing never completes a full revolution. Shigley's life model assumes continuous rotation, so the hours above are an idealisation. For small arcs, the more relevant failure mode is **false brinelling** (fretting wear at fixed ball positions), not subsurface fatigue. With loads this far below C₀, fretting is expected to be minimal.

---

## 8. Results Summary

| Quantity | Value |
|---|---|
| Equivalent load, F_e | 6.6 N |
| Design load, a_f F_e | 7.9 N |
| L₁₀ rating life | 7.2 × 10¹³ rev |
| L₉₉ life (R = 0.99) | 1.6 × 10¹³ rev (≈ 1.3 × 10¹¹ h at 2 rev/min) |
| Required C₁₀ for 8 kh, R = 0.99 | 13 N (available 3300 N, margin ≈ 255) |
| Static safety factor, s₀ | ≈ 197 |

---

## 9. Conclusions

1. Under a 6.6 N radial load the KP08 shoulder bearing runs at about **0.2 %** of its dynamic rating. Rolling-contact fatigue does not limit the design.
2. The bearing could carry about **255 times** the design load and still meet an 8 kh, 99 %-reliability life. Even with the entire arm load on this one bearing, the margin stays above 80.
3. The bearing is heavily oversized for load. It was chosen for an 8 mm shaft fit, alignment tolerance and low cost, not for capacity.
4. The practical limits are non-fatigue factors: set-screw shaft retention, play in the self-aligning housing seat (which affects positioning accuracy at the gripper), and fretting under small-angle oscillation.

---

**Reference:** Budynas, R.G. & Nisbett, J.K. (2015). *Shigley's Mechanical Engineering Design*, 10th ed. McGraw-Hill. Ch. 11: Eq. (11–1), (11–3), (11–9), (11–10), (11–12); Tables 11–1, 11–4, 11–5.
**Bearing data:** Bearings Direct, KP08 Mini Zinc Pillow Block (KP8UU), C₁₀ = 742 lbf, C₀ = 292 lbf; Mädler, KP 08 Ball Pillow Block (PN 62560800), C₁₀ = 4.2 kN, C₀ = 1.6 kN.
