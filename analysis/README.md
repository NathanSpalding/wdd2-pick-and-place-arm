# Analysis

| Folder | Contents |
|---|---|
| `torque/` | `WDD2_torque_analysis.m` (MATLAB): worst-case joint torques from the URDF, servo utilisation and payload capacity. Needs `cad/urdf/WDD2_arm.urdf`. Key findings in the text files. |
| `bearings/` | KP08 bearing life for the base and shoulder (Shigley, Ch. 11). |
| `figures/` | Python scripts that made the report's test figures. `fig_tests.py` reads `test-data/` directly; the others read the GUI exports (set `WDD2_DATA` to their folder, see `WDD2_figures_notes.md`). Needs numpy, matplotlib and openpyxl. |
