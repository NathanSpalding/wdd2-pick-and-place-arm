"""Figure: one pick-and-place cycle over time (report Section 4.7) -> test-cycle-timeline.png

Data: gui_export_8.7_500g_arm_export_20261013_221739.xlsx, sheet 'Data log' (13 Oct 2026, 500 g).
Cycle shown: counted 500 g trial 9 (GUI cycle 11, cube released at t = 2635.9 s in the log), chosen
because its base overshoot, cycle time, place error and holding current are all close to the median
of the ten counted 500 g cycles.
Phase boundaries come from the logged target position (target_x/y/z_mm) and holding_cube.
Base 'commanded' angle = the angle actually sent to the ESP: the GUI pointed the real base at
the model tool point (BASE_AIM_AT_TIP), i.e. atan2(-tip_y, tip_x). The base_cmd_deg column is the
model's own base angle, which is 1.36 deg off because of the model's 7 mm tool offset.
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
JOINT_COL = {'base': '#2a78d6', 'shoulder': '#1baf7a', 'elbow': '#6250d6', 'wrist': '#e87ba4'}

DATA = os.environ.get('WDD2_DATA', '.')
SRC = os.path.join(DATA, 'gui_export_8.7_500g_arm_export_20261013_221739.xlsx')
RELEASE_T = 2635.9        # trial 9 release time in the log (s)
THRESH = 0.04             # grasp-detection threshold (A)

d = pd.read_excel(SRC, sheet_name='Data log')
t_all = d.time_s.values
# the cycle = the 'Sequence' run that contains the release
i_rel = np.searchsorted(t_all, RELEASE_T)
seq = (d['mode'] == 'Sequence').values
a = i_rel
while a > 0 and seq[a - 1]:
    a -= 1
b = i_rel
while b < len(d) - 1 and seq[b + 1]:
    b += 1
c = d.iloc[a - 25:b + 50].reset_index(drop=True)      # 0.5 s before, 1 s after
t0 = d.time_s.values[a]
t = c.time_s.values - t0

# phase boundaries from the logged target and holding_cube
T = c[['target_x_mm', 'target_y_mm', 'target_z_mm']].values
tchg = [t[i] for i in range(1, len(c)) if np.any(np.abs(T[i] - T[i - 1]) > 0.05) and t[i] >= 0]
h = c.holding_cube.values
grab = t[np.where(np.diff(h) == 1)[0][0] + 1]
rel = t[np.where(np.diff(h) == -1)[0][0] + 1]
end = d.time_s.values[b] - t0
# expected order: above-right, descend, (grab), lift, above-left, descend, (release), lift, home
up_r, dn_r, lift_r, sw_l, dn_l, lift_l, home = tchg[:7]
phases = [('Swing', 0, dn_r), ('Down', dn_r, grab), ('Grip', grab, lift_r), ('Up', lift_r, sw_l),
          ('Swing', sw_l, dn_l), ('Down', dn_l, rel), ('Rel.', rel, lift_l), ('Up', lift_l, home),
          ('Return', home, end)]

# commanded base = angle sent to the ESP
base_sent = np.degrees(np.arctan2(-c.tip_y_mm.values, c.tip_x_mm.values))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.3, 3.5), sharex=True,
                               gridspec_kw={'height_ratios': [2.3, 1]})
for k, (name, x0, x1) in enumerate(phases):
    for ax in (ax1, ax2):
        if k % 2 == 0:
            ax.axvspan(x0, x1, color='#f3f2ef', lw=0, zorder=0)
    ax1.text((x0 + x1) / 2, 1.0, name, transform=ax1.get_xaxis_transform(), ha='center',
             va='bottom', fontsize=7, color='#52514e')
for j in ['base', 'shoulder', 'elbow', 'wrist']:
    cmd = base_sent if j == 'base' else c[f'{j}_cmd_deg'].values
    ax1.plot(t, cmd, color=JOINT_COL[j], lw=0.8, ls='--')
    ax1.plot(t, c[f'{j}_meas_deg'].values, color=JOINT_COL[j], lw=1.4, label=j.capitalize())
ax1.set_ylabel('Angle (°)')
from matplotlib.lines import Line2D
hs, ls = ax1.get_legend_handles_labels()
hs += [Line2D([], [], color=GREY, lw=1.4), Line2D([], [], color=GREY, lw=0.8, ls='--')]
ls += ['Measured', 'Commanded']
ax1.legend(hs, ls, loc='upper center', bbox_to_anchor=(0.5, -0.02), ncol=6, fontsize=8,
           handlelength=1.6, columnspacing=1.0)
# one-sample encoder read glitch on the shoulder (logged as is, not filtered out)
g = np.argmin(c.shoulder_meas_deg.values - c.shoulder_cmd_deg.values)
ax1.annotate('encoder glitch (1 sample)', xy=(t[g], c.shoulder_meas_deg.values[g]),
             xytext=(t[g] + 0.35, 20), fontsize=7, color='#52514e',
             arrowprops=dict(arrowstyle='-', color=GREY, lw=0.6))
ax1.set_title('(a) Joint angles', fontsize=10, loc='left', pad=12)
ax1.tick_params(labelbottom=False)

ax2.plot(t, c.gripper_I_meas_A.values, color=BLUE, lw=1.2, label='Measured')
ax2.axhline(THRESH, color=GREY, ls='--', lw=0.9, label='Detection threshold (0.04 A)')
ax2.set_ylabel('Current (A)')
ax2.set_xlabel('Time (s)')
ax2.set_title('(b) Gripper current', fontsize=10, loc='left')
ax2.legend(loc='upper left', fontsize=8)
ax2.set_xlim(t[0], t[-1])
fig.tight_layout(h_pad=0.4)
fig.savefig('test-cycle-timeline.png')

# ---- numbers for the caption and text ----
held = (t > grab + 0.5) & (t < rel)
print(f'cycle: trial 9 (GUI cycle 11), log t0 = {t0:.2f} s, sequence run {end:.2f} s')
print('phase boundaries (s):', ', '.join(f'{n} {x0:.2f}-{x1:.2f}' for n, x0, x1 in phases))
print(f'grab command at {grab:.2f} s, release at {rel:.2f} s')
gw = (t >= grab) & (t < grab + 0.5)
rw = (t >= rel) & (t < rel + 0.5)
print(f'gripper current: peak at grasp {c.gripper_I_meas_A.values[gw].max():.3f} A, peak at release {c.gripper_I_meas_A.values[rw].max():.3f} A, holding median '
      f'{np.median(c.gripper_I_meas_A.values[held]):.3f} A')
for name, x0, x1 in (('swing to pick (+45)', 0, dn_r), ('swing to place (-45)', sw_l, dn_l),
                     ('return home', home, t[-1])):
    w = (t >= x0) & (t < x1)
    tgt = base_sent[w][-1]
    cmd_over = max(0.0, np.max((base_sent[w] - tgt) * np.sign(tgt - base_sent[w][0])))
    start = c.base_meas_deg.values[w][0]
    s = np.sign(tgt - start)
    over = max(0.0, np.max((c.base_meas_deg.values[w] - tgt) * s))
    print(f'base overshoot, {name}: target {tgt:+.2f} deg, measured past target {over:.2f} deg, '
          f'of which the GUI command itself went {cmd_over:.2f} deg past')
dt = np.diff(c.time_s.values)
print(f'log interval in this cycle: median {1000 * np.median(dt):.0f} ms, max {1000 * dt.max():.0f} ms')
