import numpy as np
import pinocchio as pin
from robot_descriptions import panda_description

# Load the URDF into Pinocchio
model = pin.buildModelFromUrdf(panda_description.URDF_PATH)
data = model.createData()

print("Number of joint variables:", model.nq)
print("Joints:")
for name in model.names:
    print("  ", name)

# Pick the end-effector frame
frame_name = "panda_hand" if model.existFrame("panda_hand") else model.frames[-1].name
frame_id = model.getFrameId(frame_name)
print("End-effector frame:", frame_name)

# Forward kinematics at the neutral pose (all joints at their middle value)
q = pin.neutral(model)
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

position = data.oMf[frame_id].translation
print("End-effector position (x, y, z) in metres:", np.round(position, 3))