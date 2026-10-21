import time

import numpy as np
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description

urdf = load_robot_description("panda_description")

server = viser.ViserServer()
viser_urdf = ViserUrdf(server, urdf_or_path=urdf)

# Joint limits, read from the URDF: {joint name: (lower, upper)}
limits = viser_urdf.get_actuated_joint_limits()

sliders = []
initial = []
for name, (lower, upper) in limits.items():
    lower = lower if lower is not None else -np.pi
    upper = upper if upper is not None else np.pi
    start = float(np.clip(0.0, lower, upper))
    slider = server.gui.add_slider(
        name, min=lower, max=upper, step=0.01, initial_value=start
    )
    sliders.append(slider)
    initial.append(start)


def update(_):
    viser_urdf.update_cfg(np.array([s.value for s in sliders]))


for s in sliders:
    s.on_update(update)

viser_urdf.update_cfg(np.array(initial))

print("Open http://localhost:8080 in your browser")
print("Press Ctrl+C here to stop")
while True:
    time.sleep(1)