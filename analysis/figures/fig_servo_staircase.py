"""Figure: servo step 'staircase' against the encoder (report Section 4.3) -> test-servo-staircase.png

Data: day-1 bring-up-sketch logs 8.3_deadband_shoulder.txt and 8.3_deadband_wrist.txt (12 Oct 2026).
Each 'joint (GPIO n) -> P us' line is a commanded pulse; the monitor lines that follow give the raw
AS5600 count. The settled count for a pulse is the median of the last 10 monitor samples before the
next command. Encoder angle change = DIR x (count - start count) x 360/4096 (DIR = -1 for both joints).
Both joints were first set above the start pulse and then stepped down 1 us at a time, so the
first steps are a reversal.
The elbow is not plotted: its stepping-down log covers only 6 us (1582 -> 1576 us) and shows a
1-count creep before the reversal, so it does not give a clean staircase. The wrist is used as
the second series instead.
"""
import os
import re
import numpy as np
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
CNT = 360.0 / 4096


def staircase(path, joint, start_us, dirn=-1):
    """(pulse, settled raw count) pairs from the last command at start_us onwards."""
    seq, cur, raws = [], None, []
    for line in open(path, errors='ignore'):
        m = re.search(r'(\w+) \(GPIO \d+\) -> (\d+) us', line)
        if m:
            if cur is not None:
                seq.append((cur, raws))
            cur, raws = (m.group(1), int(m.group(2))), []
            continue
        m = re.search(r'^\s*(\w+)\s+-?[\d.]+ deg \(raw\s+(\d+)\)', line)
        if m and cur is not None and m.group(1) == cur[0]:
            raws.append(int(m.group(2)))
    if cur is not None:
        seq.append((cur, raws))
    seq = [(p, r) for (j, p), r in seq if j == joint and r]
    i0 = max(i for i, (p, _) in enumerate(seq) if p == start_us)
    pulses = np.array([p for p, _ in seq[i0:]])
    counts = np.array([np.median(r[-10:]) for _, r in seq[i0:]])
    ang = dirn * (counts - counts[0]) * CNT
    return pulses, ang


JOINTS = [  # name, file, start pulse (us), calibration slope K (deg/us), colour
    ('Shoulder', '8.3_deadband_shoulder.txt', 1558, -0.1292, BLUE),
    ('Wrist', '8.3_deadband_wrist.txt', 1548, -0.1361, ORANGE),
]

fig, ax = plt.subplots(figsize=(6.3, 2.9))
results = {}
for name, fname, p0, K, col in JOINTS:
    p, a = staircase(os.path.join(DATA, fname), name.lower(), p0)
    x = p0 - p                                        # pulse decrease from the start (us)
    ax.step(x, a, where='post', color=col, lw=1.5, label=f'{name}: encoder')
    ax.plot(x, a, 'o', color=col, ms=2.2)
    xi = np.linspace(0, x.max(), 50)
    ax.plot(xi, -K * xi, color=col, lw=0.8, ls=':', alpha=0.9)
    moved = np.where(np.abs(a) > 1.5 * CNT)[0]        # first change of 2 counts or more
    first = x[moved[0]]
    jumps = np.diff(a)
    big = jumps[np.abs(jumps) > 1.5 * CNT]
    jump_at = x[1:][np.abs(jumps) > 1.5 * CNT]
    results[name] = dict(deadband_us=first, deadband_deg=first * abs(K), jumps_deg=big,
                         jump_at_us=jump_at, final=(x[-1], a[-1], -K * x[-1]))
    ax.annotate('', xy=(0, -0.25 if name == 'Shoulder' else -0.5),
                xytext=(first, -0.25 if name == 'Shoulder' else -0.5),
                arrowprops=dict(arrowstyle='<->', color=col, lw=0.8, shrinkA=0, shrinkB=0))
    if name == 'Shoulder':
        ax.text(first + 0.4, -0.25, f'reversal deadband {first:.0f} µs', ha='left', va='center',
                fontsize=7.5, color=col)
    else:
        ax.text(first + 0.4, -0.5, f'reversal deadband {first:.0f} µs', ha='left', va='center',
                fontsize=7.5, color=col)
ax.plot([], [], color=GREY, lw=0.8, ls=':', label='Ideal response from calibration slope K')
ax.set_xlabel('Pulse width decrease from start (µs)')
ax.set_ylabel('Joint angle change (°)')
ax.set_ylim(-0.85, 5.3)
ax.set_xlim(-0.5, None)
ax.legend(loc='upper left', fontsize=8)
fig.tight_layout()
fig.savefig('test-servo-staircase.png')

for name, r in results.items():
    print(f"{name}: reversal deadband {r['deadband_us']:.0f} us = {r['deadband_deg']:.3f} deg (x |K|); "
          f"jumps {', '.join(f'{j:.3f}' for j in r['jumps_deg'])} deg at "
          f"{', '.join(f'{u:.0f}' for u in r['jump_at_us'])} us; "
          f"after {r['final'][0]:.0f} us: encoder {r['final'][1]:.2f} deg vs ideal {r['final'][2]:.2f} deg")
print(f'encoder resolution {CNT:.4f} deg/count')
