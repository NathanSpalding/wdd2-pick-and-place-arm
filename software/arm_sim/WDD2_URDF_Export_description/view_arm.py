"""
View and check the WDD2 arm URDF.

Put this file in the WDD2_URDF_Export_description folder (next to urdf/ and meshes/),
then run:   python view_arm.py
Open http://localhost:8080 - one slider per joint.

It also prints:
  - joint names, limits, and link masses (from Pinocchio)
  - the height the model sits at, and the WORLD_Z value to put in the URDF so the
    bottom of the base rests on the floor (z = 0)
"""

import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import viser
import yourdfpy
from viser.extras import ViserUrdf

HERE = Path(__file__).resolve().parent
URDF_PATH = HERE / "urdf" / "WDD2_arm.urdf"

# ---------- Pinocchio: joints, limits, masses ----------
model = pin.buildModelFromUrdf(str(URDF_PATH))
print("\nJoints (limits in degrees):")
for i in range(1, model.njoints):
    idx = model.joints[i].idx_q
    lo, hi = np.degrees([model.lowerPositionLimit[idx], model.upperPositionLimit[idx]])
    print(f"  {model.names[i]:10s} {lo:7.1f} to {hi:6.1f}")

print("\nLink masses:")
for i in range(1, model.njoints):
    print(f"  after {model.names[i]:10s} {model.inertias[i].mass * 1000:7.1f} g")

# ---------- Mesh bounds: where is the floor? ----------
urdf = yourdfpy.URDF.load(str(URDF_PATH))
bounds = urdf.scene.bounds          # [[xmin, ymin, zmin], [xmax, ymax, zmax]] in metres
zmin = bounds[0][2]
print(f"\nModel bounds (m): min {np.round(bounds[0], 3)}, max {np.round(bounds[1], 3)}")
if abs(zmin) > 0.002:
    print(f"The lowest point is at z = {zmin:.4f} m.")
    print(f"To put the base on the floor, set WORLD_Z in the URDF to {-zmin:.4f}")
    print("(the last number of xyz in the world_to_base joint), then run this again.")
else:
    print("The base already sits on the floor (z = 0).")

# ---------- Viewer with sliders ----------
server = viser.ViserServer()
server.scene.set_up_direction("+z")
server.scene.add_grid("/ground", width=1.5, height=1.5, cell_size=0.05,
                      position=(0.0, 0.0, 0.0))
viser_urdf = ViserUrdf(server, urdf_or_path=urdf)

sliders = []
for name, (lower, upper) in viser_urdf.get_actuated_joint_limits().items():
    lower = -np.pi if lower is None else lower
    upper = np.pi if upper is None else upper
    sliders.append(server.gui.add_slider(
        f"{name} (rad)", min=lower, max=upper, step=0.01,
        initial_value=float(np.clip(0.0, lower, upper))))


def update(_=None):
    viser_urdf.update_cfg(np.array([s.value for s in sliders]))


for s in sliders:
    s.on_update(update)
update()

print("\nOpen http://localhost:8080  (Ctrl+C to stop)")
while True:
    time.sleep(1)
