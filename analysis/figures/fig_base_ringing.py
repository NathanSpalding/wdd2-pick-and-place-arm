"""Figure: base ringing after the 90 deg swing to the place station (report Section 4.9) -> test-base-ringing.png

Original profile: gui_export_8.7_500g_arm_export_20261013_221739.xlsx (13 Oct 2026), the ten counted
  500 g cycles of test 8.7. GUI streaming J at 50 Hz, 80 ms base S-curve, continuous base trim.
Final profile: arm_export_20261017_005005.xlsx (17 Oct 2026), the four arrivals with the 500 g cube
  held. Waypoint mode (ESP plans the move), 200 ms base S-curve, 60 deg/s base cap, one-shot trim.
Both: GUI data log, nominal 50 Hz (median interval 22 ms); the base angle is the latest AS5600
  reading the GUI had received (A lines, 50 Hz).
t = 0 is the first sample at which the measured base is within 1 deg of the -45 deg target.
Error = measured - target; negative = past the target (further round than -45 deg).
Several things changed between the two profiles at once (stream vs waypoint, S-curve, speed cap,
trim mode), so the difference cannot be credited to the S-curve alone.
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams.update({
    'font.family': 'STIXGeneral', 'mathtext.fontset': 'stix', 'font.size': 10,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.edgecolor': '#52514e', 'axes.linewidth': 0.8,
    'xtick.color': '#52514e', 'ytick.color': '#52514e',
    'axes.grid': True, 'grid.color': '#e4e3df', 'grid.linewidth': 0.6,
    'lines.linewidth': 1.6, 'legend.frameon': False, 'savefig.dpi': 300})
BLUE, ORANGE, GREY = '#2a78d6', '#eb6834', '#8a8984'
DATA = os.environ.get('WDD2_DATA', '.')
TARGET = -45.0
DEADBAND = 0.4
WIN = 3.0


def arrivals(path, t_lo=-np.inf, t_hi=np.inf, need_cube=True):
    d = pd.read_excel(path, sheet_name='Data log')
    t, m = d.time_s.values, d.base_meas_deg.values
    tgt = np.degrees(np.arctan2(-d.target_y_mm.values, d.target_x_mm.values))
    idx = np.where((np.abs(tgt - TARGET) < 0.5) & (np.abs(m - TARGET) < 1.0) & (t > t_lo) & (t < t_hi))[0]
    firsts = []
    for i in idx:
        if not firsts or t[i] - t[firsts[-1]] > 5:
            firsts.append(i)
    out = []
    for j in firsts:
        if need_cube and d.holding_cube.values[j] != 1:
            continue
        w = (t >= t[j]) & (t <= t[j] + WIN)
        out.append((t[w] - t[j], m[w] - TARGET, np.diff(t[w])))
    return out


def metrics(tt, e):
    over = max(0.0, -e.min())                         # past the target (more negative than -45)
    ring = np.ptp(e[(tt >= 0.5) & (tt <= 2.0)])
    final = e[(tt >= 2.5)].mean()
    out = np.where(np.abs(e - final) > 0.3)[0]
    settle = tt[out[-1]] if len(out) else 0.0
    inband = np.where(np.abs(e) > DEADBAND)[0]
    band = tt[inband[-1]] if len(inband) else 0.0
    return over, ring, settle, final, band


orig = arrivals(os.path.join(DATA, 'gui_export_8.7_500g_arm_export_20261013_221739.xlsx'), 2240, 2700)
final = arrivals(os.path.join(DATA, 'arm_export_20261017_005005.xlsx'))

fig, ax = plt.subplots(figsize=(6.3, 2.8))
ax.axhspan(-DEADBAND, DEADBAND, color=GREY, alpha=0.18, lw=0, label='±0.4° base trim deadband')
for runs, col, name in ((orig, ORANGE, 'Original (13 Oct, 80 ms S-curve, GUI stream)'),
                        (final, BLUE, 'Final (17 Oct, 200 ms S-curve, 60 °/s cap, waypoint)')):
    rings = [metrics(tt, e)[1] for tt, e, _ in runs]
    rep = int(np.argsort(rings)[len(rings) // 2])     # median ring = representative
    for k, (tt, e, _) in enumerate(runs):
        ax.plot(tt, e, color=col, lw=1.5 if k == rep else 0.6, alpha=1.0 if k == rep else 0.35,
                label=f'{name}, n = {len(runs)}' if k == rep else None, zorder=3 if k == rep else 2)
ax.axhline(0, color=GREY, lw=0.6)
ax.set_xlim(0, WIN)
ax.set_xlabel('Time from arrival at the place station (s)')
ax.set_ylabel('Base error (°)')
ax.legend(fontsize=7.5, loc='lower right')
fig.tight_layout()
fig.savefig('test-base-ringing.png')

for runs, name in ((orig, 'original'), (final, 'final')):
    M = np.array([metrics(tt, e) for tt, e, _ in runs])
    gaps = np.concatenate([g for _, _, g in runs])
    print(f'{name}: n = {len(runs)} arrivals; log interval median {1000 * np.median(gaps):.0f} ms, '
          f'max {1000 * gaps.max():.0f} ms')
    for lab, col in (('overshoot past target', 0), ('peak-to-peak 0.5-2.0 s', 1),
                     ('settling time to +-0.3 deg of final', 2), ('final error (2.5-3.0 s)', 3),
                     ('last time outside +-0.4 deg band', 4)):
        v = M[:, col]
        print(f'   {lab}: mean {v.mean():.2f}, range {v.min():.2f} to {v.max():.2f}')
