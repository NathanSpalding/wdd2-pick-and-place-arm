"""Figure: pulse-to-angle calibration fit for the shoulder (report Section 3.5.4) -> ctrl-calibration-fit.png

Data: WDD2_bringup_data_2026-10-09.xlsx, sheets 'Shoulder', 'Elbow', 'Wrist' (bring-up, 9 Oct 2026).
Each row is a commanded pulse and the settled encoder angle (last sample before the next command).
Rows marked 'no (at stop)' were pressed against a mechanical stop and are left out of the fit,
as in the original fit. The fit is refitted here by least squares to check the published P0 and K.
The base sweep is not in this file (the base was calibrated after the bring-up night) and is
therefore not used; no points are reconstructed from the fit.
Note: 'measured angle' is the joint's own encoder, not an independent reference.
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
SRC = os.path.join(DATA, 'WDD2_bringup_data_2026-10-09.xlsx')


def load(sheet):
    raw = pd.read_excel(SRC, sheet_name=sheet, header=None)
    hdr = raw.index[raw[0].astype(str) == 'Step'][0]
    df = raw.iloc[hdr + 1:].copy()
    df.columns = raw.iloc[hdr].tolist()
    df = df[pd.to_numeric(df['Pulse (us)'], errors='coerce').notna()].copy()
    for col in ['Pulse (us)', 'Measured angle (deg)']:
        df[col] = df[col].astype(float)
    df['used'] = df['Used in fit'].astype(str).str.startswith('yes')
    return df


def fit(df):
    u = df[df.used]
    k, b = np.polyfit(u['Pulse (us)'], u['Measured angle (deg)'], 1)
    res = u['Measured angle (deg)'] - (b + k * u['Pulse (us)'])
    ss = np.sum((u['Measured angle (deg)'] - u['Measured angle (deg)'].mean()) ** 2)
    return dict(k=k, b=b, P0=-b / k, rms=np.sqrt(np.mean(res ** 2)), maxres=np.max(np.abs(res)),
                r2=1 - np.sum(res ** 2) / ss, n=len(u), n_out=int((~df.used).sum()))


stats = {j: fit(load(j)) for j in ['Shoulder', 'Elbow', 'Wrist']}

sh = load('Shoulder')
f = stats['Shoulder']
u, o = sh[sh.used], sh[~sh.used]
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(6.3, 2.8), gridspec_kw={'width_ratios': [1.15, 1]})
pp = np.linspace(850, 1900, 50)
ax1.plot(pp, f['b'] + f['k'] * pp, color=ORANGE, lw=1.2,
         label=f"Fit: $K$ = {f['k']:.4f} °/µs, $P_0$ = {f['P0']:.0f} µs")
ax1.plot(u['Pulse (us)'], u['Measured angle (deg)'], 'o', color=BLUE, ms=3.5, label='Measured (encoder)')
ax1.plot(o['Pulse (us)'], o['Measured angle (deg)'], 'o', mfc='none', mec=GREY, ms=3.5,
         label='At mechanical stop (not fitted)')
ax1.set_xlabel('Pulse width (µs)')
ax1.set_ylabel('Shoulder angle (°)')
ax1.set_title('(a) Shoulder angle against pulse width', fontsize=10, loc='left')
ax1.legend(fontsize=7.5, loc='lower left')

resid = u['Measured angle (deg)'] - (f['b'] + f['k'] * u['Pulse (us)'])
ax2.axhspan(-0.5, 0.5, color=GREY, alpha=0.18, lw=0, label='±0.5° reference uncertainty')
ax2.axhline(0, color=GREY, lw=0.6)
# approach direction: was the pulse raised or lowered to reach this point? (shows the hysteresis)
dP = sh['Pulse (us)'].diff().loc[u.index].fillna(0).values
up = dP > 0
ax2.plot(u['Measured angle (deg)'][up], resid[up], 'o', color=BLUE, ms=3.5, label='Pulse raised to reach point')
ax2.plot(u['Measured angle (deg)'][~up], resid[~up], 'o', mfc='white', mec=BLUE, ms=3.5,
         label='Pulse lowered to reach point')
ax2.set_xlabel('Shoulder angle (°)')
ax2.set_ylabel('Residual (°)')
ax2.set_title('(b) Fit residuals', fontsize=10, loc='left')
ax2.legend(fontsize=7, loc='upper center', bbox_to_anchor=(0.55, 1.0))
fig.tight_layout()
fig.savefig('ctrl-calibration-fit.png')

print('shoulder residuals by approach: pulse raised', np.round(resid[up].values, 2),
      '| pulse lowered', np.round(resid[~up].values, 2))
for j, s in stats.items():
    print(f"{j}: n = {s['n']} points (+{s['n_out']} at stops, excluded), K = {s['k']:.4f} deg/us, "
          f"P0 = {s['P0']:.1f} us, RMS residual {s['rms']:.2f} deg, largest {s['maxres']:.2f} deg, "
          f"R^2 = {s['r2']:.5f}")
