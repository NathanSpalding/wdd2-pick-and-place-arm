import time

import numpy as np
import pinocchio as pin
import pink
import viser
from pink import solve_ik
from pink.tasks import FrameTask
from viser.extras import ViserUrdf
from robot_descriptions import panda_description
from robot_descriptions.loaders.yourdfpy import load_robot_description

# --- Robot maths (Pinocchio) ---
model = pin.buildModelFromUrdf(panda_description.URDF_PATH)
data = model.createData()
q0 = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02])
configuration = pink.Configuration(model, data, q0)

# --- Robot picture (viewer) ---
urdf = load_robot_description("panda_description")
server = viser.ViserServer()
viser_urdf = ViserUrdf(server, urdf_or_path=urdf)
viewer_joint_names = list(viser_urdf.get_actuated_joint_limits().keys())


def show(q):
    cfg = [q[model.joints[model.getJointId(n)].idx_q] for n in viewer_joint_names]
    viser_urdf.update_cfg(np.array(cfg))


# --- IK task: hand position only ---
frame_name = "panda_hand"
task = FrameTask(frame_name, position_cost=1.0, orientation_cost=0.5)

start_pos = configuration.get_transform_frame_to_world(frame_name).translation
target = server.scene.add_transform_controls(
    "/target", scale=0.2, position=tuple(start_pos)
)

# --- Route state ---
waypoints = []   # list of 3D positions
markers = []     # red spheres shown in the viewer
state = {"playing": False, "index": 0, "arrived_at": None, "started_at": 0.0}

# --- Buttons ---
add_btn = server.gui.add_button("Add waypoint")
play_btn = server.gui.add_button("Play route")
stop_btn = server.gui.add_button("Stop")
clear_btn = server.gui.add_button("Clear route")
status = server.gui.add_text("Status", initial_value="Idle", disabled=True)


def go_to(i):
    state["index"] = i
    state["arrived_at"] = None
    state["started_at"] = time.time()
    target.position = tuple(waypoints[i])
    status.value = f"Moving to waypoint {i + 1} of {len(waypoints)}"


@add_btn.on_click
def _(_):
    pos = np.array(target.position)
    waypoints.append(pos)
    marker = server.scene.add_icosphere(
        f"/waypoints/{len(waypoints)}",
        radius=0.02,
        color=(255, 80, 80),
        position=tuple(pos),
    )
    markers.append(marker)
    status.value = f"{len(waypoints)} waypoints saved"


@play_btn.on_click
def _(_):
    if waypoints:
        state["playing"] = True
        go_to(0)


@stop_btn.on_click
def _(_):
    state["playing"] = False
    status.value = "Stopped"


@clear_btn.on_click
def _(_):
    state["playing"] = False
    for m in markers:
        m.remove()
    markers.clear()
    waypoints.clear()
    status.value = "Route cleared"


show(configuration.q)
print("Open http://localhost:8080")
print("Drag the target, click 'Add waypoint', repeat, then 'Play route'")
print("Press Ctrl+C here to stop")

dt = 0.02
while True:
    if state["playing"] and waypoints:
        i = state["index"]
        hand = configuration.get_transform_frame_to_world(frame_name).translation
        close_enough = np.linalg.norm(hand - waypoints[i]) < 0.01
        timed_out = time.time() - state["started_at"] > 10.0

        if close_enough or timed_out:
            if state["arrived_at"] is None:
                state["arrived_at"] = time.time()
            if time.time() - state["arrived_at"] > 0.5:
                if i + 1 < len(waypoints):
                    go_to(i + 1)
                else:
                    state["playing"] = False
                    status.value = "Route finished"

    goal = pin.SE3(np.diag([1.0, -1.0, -1.0]), np.array(target.position))
    task.set_target(goal)
    velocity = solve_ik(configuration, [task], dt, solver="quadprog", damping=1e-3)
    configuration.integrate_inplace(velocity, dt)
    show(configuration.q)
    time.sleep(dt)