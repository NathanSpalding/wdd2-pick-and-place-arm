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

# --- Load the robot in Pinocchio (for the maths) ---
model = pin.buildModelFromUrdf(panda_description.URDF_PATH)
data = model.createData()

# A sensible starting pose (7 arm joints, then 2 fingers)
q0 = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02])
configuration = pink.Configuration(model, data, q0)

# --- Load the same robot in the viewer (for the picture) ---
urdf = load_robot_description("panda_description")
server = viser.ViserServer()
viser_urdf = ViserUrdf(server, urdf_or_path=urdf)
viewer_joint_names = list(viser_urdf.get_actuated_joint_limits().keys())


def show(q):
    """Send Pinocchio's joint values to the viewer, matched by joint name."""
    cfg = [q[model.joints[model.getJointId(n)].idx_q] for n in viewer_joint_names]
    viser_urdf.update_cfg(np.array(cfg))


# --- IK task: move the hand to a target position ---
frame_name = "panda_hand"
task = FrameTask(frame_name, position_cost=1.0, orientation_cost=0.0)

# --- Draggable target handle ---
start_pos = configuration.get_transform_frame_to_world(frame_name).translation
target = server.scene.add_transform_controls(
    "/target", scale=0.2, position=tuple(start_pos)
)

show(configuration.q)
print("Open http://localhost:8080 and drag the target handle")
print("Press Ctrl+C here to stop")

dt = 0.02  # 50 updates per second
while True:
    goal = pin.SE3(np.eye(3), np.array(target.position))
    task.set_target(goal)
    velocity = solve_ik(configuration, [task], dt, solver="quadprog", damping=1e-3)
    configuration.integrate_inplace(velocity, dt)
    show(configuration.q)
    time.sleep(dt)