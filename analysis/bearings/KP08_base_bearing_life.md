# KP08 Base Bearing – Rolling-Contact Bearing Life Analysis

**Project:** WDD2 pick-and-place robotic arm (Mechatronic Project 478)
**Component:** KP08 pillow-block insert bearing in the base coupling
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
| Axial load, F_a | **15.5 N** | Full arm (1.124 kg) + turntable (0.457 kg), payload 500 g, arm fully extended |
| Radial load, F_r | 0 N | The bearing carries the arm's weight only (design assumption) |
| Rotation factor, V | 1 | Inner ring rotates with the shaft; housing is stationary |
| Load-application factor, a_f | **1.2** | Table 11–5: machinery with light impact (servo start/stop), 1.2–1.5 |
| Weibull parameters | x₀ = 0.02, (θ − x₀) = 4.439, b = 1.483 | Shigley typical ball-bearing values (p. 601, Ex. 11–3) |
| Design reliability, R_D | 0.99 | Single critical support bearing |
| Duty cycle | 0.5 rev per 15 s cycle → **n ≈ 2 rev/min** average | 90° base move out and back per pick-and-place cycle |

Oscillating motion of ±90° is large enough to roll every ball over a fresh track. It is therefore treated as equivalent continuous rotation at the same total angular travel.

---

## 2. Equivalent Radial Load — Eq. (11–12), Table 11–1

F_a / C₀ = 15.5 / 1300 = 0.0119 < 0.014 → use the first row of Table 11–1:

e = 0.19, X₂ = 0.56, Y₂ = 2.30

With F_r = 0, F_a /(V F_r) → ∞ > e, so:

F_e = X₂ V F_r + Y₂ F_a = 0.56(1)(0) + 2.30(15.5) = **35.7 N**

Design load including the application factor:

F_D = a_f F_e = 1.2 × 35.7 = **42.8 N**

---

## 3. Rating Life (90 % reliability) — Eq. (11–3)

L₁₀ = (C₁₀ / F_D)ᵃ × L_R = (3300 / 42.8)³ × 10⁶ = (77.1)³ × 10⁶

**L₁₀ ≈ 4.6 × 10¹¹ rev**

At n = 2 rev/min: l₁₀ = L₁₀ / (60 n) ≈ **3.8 × 10⁹ h**

---

## 4. Life at 99 % Reliability — Eq. (11–9) rearranged

x_D = (C₁₀ / F_D)ᵃ · [x₀ + (θ − x₀)(ln 1/R_D)^(1/b)]

[x₀ + (θ − x₀)(ln 1/0.99)^(1/1.483)] = 0.02 + 4.439(0.01005)^0.674 = **0.220**

L₉₉ = 4.59 × 10⁵ × 0.220 × 10⁶ ≈ **1.0 × 10¹¹ rev**  →  l₉₉ ≈ **8.4 × 10⁸ h**

---

## 5. Check Against Required Life — Eq. (11–10), Table 11–4

Requirement: Table 11–4, *machines for intermittent service where reliable operation is of great importance*, **8 kh** at R = 0.99.

L_D = 8000 × 60 × 2 = 9.6 × 10⁵ rev  →  x_D = 0.96

C₁₀,req = a_f F_e [x_D / (x₀ + (θ − x₀)(1 − R_D)^(1/b))]^(1/3) = 42.8 × (0.96 / 0.220)^(1/3) = **70 N**

C₁₀,available / C₁₀,req = 3300 / 70 ≈ **47**

---

## 6. Static Load Check

Pure axial load on a deep-groove bearing, taking P₀ = F_a conservatively:

s₀ = C₀ / P₀ = 1300 / 15.5 ≈ **84**  (typical minimum 1–2 for smooth operation)

---

## 7. Sensitivity — Overturning Moment Carried as Radial Load

With the arm extended, the overturning moment at the base is **4.60 N·m**. If this moment were carried by a pair of bearings a distance *d* apart, each would see F_r = M / d:

| Spacing d | F_r per bearing | F_a /(V F_r) | F_e | L₉₉ (rev) | l₉₉ at 2 rev/min | s₀ = C₀ / F_r |
|---|---|---|---|---|---|---|
| 30 mm | 153 N | 0.10 ≤ e | 153 N | 1.3 × 10⁹ | 1.1 × 10⁷ h | 8.5 |
| 40 mm | 115 N | 0.13 ≤ e | 115 N | 3.0 × 10⁹ | 2.5 × 10⁷ h | 11.3 |
| 60 mm | 77 N | 0.20 > e | 79 N | 9.4 × 10⁹ | 7.8 × 10⁷ h | 17.0 |

(a_f = 1.2 applied to all cases.)

Note: the KP08 housing is **self-aligning** (spherical seat). A single KP08 cannot resist a moment, so the overturning moment must be reacted by a second support (a second bearing or the servo output) and not by this bearing alone.

---

## 8. Results Summary

| Quantity | Value |
|---|---|
| Equivalent load, F_e | 35.7 N |
| Design load, a_f F_e | 42.8 N |
| L₁₀ rating life | 4.6 × 10¹¹ rev |
| L₉₉ life (R = 0.99) | 1.0 × 10¹¹ rev (≈ 8.4 × 10⁸ h at 2 rev/min) |
| Required C₁₀ for 8 kh, R = 0.99 | 70 N (available 3300 N, margin ≈ 47) |
| Static safety factor, s₀ | ≈ 84 |

---

## 9. Conclusions

1. Under the full 15.5 N axial load of the arm, the KP08 fatigue life far exceeds any realistic service life. Rolling-contact fatigue does **not** limit the design.
2. The bearing could carry about **47 times** the design load and still meet an 8 kh, 99 %-reliability life.
3. Even if the base overturning moment is carried as radial load by a pair of bearings 30–60 mm apart, the life stays above 10⁷ h and s₀ > 8.
4. Practical limits are therefore non-fatigue factors: lubricant degradation in the sealed insert, set-screw shaft retention, the zinc die-cast housing, and the self-aligning seat's inability to react moment.

---

**Reference:** Budynas, R.G. & Nisbett, J.K. (2015). *Shigley's Mechanical Engineering Design*, 10th ed. McGraw-Hill. Ch. 11: Eq. (11–1), (11–3), (11–9), (11–10), (11–12); Tables 11–1, 11–4, 11–5.
**Bearing data:** Bearings Direct, KP08 Mini Zinc Pillow Block (KP8UU), C₁₀ = 742 lbf, C₀ = 292 lbf; Mädler, KP 08 Ball Pillow Block (PN 62560800), C₁₀ = 4.2 kN, C₀ = 1.6 kN.
