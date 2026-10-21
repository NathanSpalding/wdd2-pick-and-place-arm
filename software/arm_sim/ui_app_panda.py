"""
Pick-and-place arm simulator: user interface
============================================

Run:   python ui_app.py
Then open http://localhost:8080 in your browser.
Stop:  press Ctrl+C in the terminal.

Tabs
  Control   - status, type in X, Y, Z (mm), grab/release the cube
  Sequence  - build (presets or custom points), run over several trials,
              save and load step-by-step tasks
  Joints    - joint angles, speeds, and speed/acceleration limits
  Currents  - simulated vs measured servo currents, and CSV export
  Report    - cycle report after each trial, and the one-file data export
  Hardware  - serial connection to the ESP, and the message format
  Setup     - display options and the workspace map

All positions shown in the interface are in millimetres.

The EMERGENCY STOP button at the top is always visible.

Until the ESP sends real readings, the "measured" currents are fictitious
(generated from the simulated values with a gain error, offset, lag, and noise).

Everything you might want to change for your own arm is in SETTINGS below.
"""

import csv
import json
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pinocchio as pin
import pink
import plotly.graph_objects as go
import viser
from pink import solve_ik
from pink.tasks import FrameTask
from viser.extras import ViserUrdf
from robot_descriptions import panda_description
from robot_descriptions.loaders.yourdfpy import load_robot_description

try:
    import serial
    import serial.tools.list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False

# =============================================================================
# SETTINGS  (change these when you switch to your own arm)
# =============================================================================
URDF_PATH = panda_description.URDF_PATH
HAND_FRAME = "panda_hand"        # frame the gripper is attached to
TCP_OFFSET_MM = 103.4            # from that frame to the gripper tip
SHOULDER_MM = (0.0, 0.0, 333.0)  # shoulder joint position
MAX_REACH_MM = 850.0             # rough reach from the shoulder

NUM_JOINTS = 7                   # arm joints (the gripper fingers are extra)
KT = 15.0                        # made-up torque constant (Nm per A)
CURRENT_LIMIT = 4.0              # made-up current limit per servo (A)
IDLE_CURRENT = 0.15              # A per servo when not loaded
SUPPLY_VOLTAGE = 6.0             # V, used for the energy figure in the report

MAX_JOINT_SPEED = 1.5            # rad/s, starting value (editable in Joints tab)
MAX_JOINT_ACCEL = 8.0            # rad/s^2, starting value (editable in Joints tab)
IN_POSITION_MM = 5.0             # "arrived" when the tip is this close (mm)

CUBE_SIZE_MM = 50.0              # edge length
CUBE_MASS = 0.5                  # kg
CUBE_HOME_MM = (500.0, 0.0)      # where the cube starts (x, y)

# Named positions of the gripper tip (x, y, z) in millimetres. "Home" is added
# automatically from the starting pose.
PRESETS_MM = {
    "Above pick": (500, 0, 200),
    "Pick": (500, 0, CUBE_SIZE_MM / 2),
    "Above place": (300, 400, 200),
    "Place": (300, 400, CUBE_SIZE_MM / 2),
}

BAUD_RATES = ["115200", "230400", "921600", "57600", "9600"]
DEMO_PORT = "Simulated ESP (demo)"
MEASURED_TIMEOUT = 0.5           # s: ESP readings older than this are ignored

WORKSPACE_STEP_MM = 40.0         # grid spacing of the workspace map
WORKSPACE_RADIUS_MM = 950.0      # how far out the map scans

DT = 0.02                        # control loop period (50 Hz)
LOG_EVERY = 5                    # log a CSV row every 5 loops (10 Hz)
BRAND = (230, 120, 40)           # accent colour for the interface
JOINT_COLOURS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
                 "#9467bd", "#8c564b", "#e377c2"]
R_DOWN = np.diag([1.0, -1.0, -1.0])  # gripper pointing straight down

HERE = Path(__file__).resolve().parent
LOG_DIR = HERE / "logs"

# ---- Internal units are metres (what Pinocchio uses). Do not edit below. ----
MM = 1000.0
TCP_OFFSET = TCP_OFFSET_MM / MM
SHOULDER = np.array(SHOULDER_MM) / MM
MAX_REACH = MAX_REACH_MM / MM
CUBE_SIZE = CUBE_SIZE_MM / MM
CUBE_HOME = (CUBE_HOME_MM[0] / MM, CUBE_HOME_MM[1] / MM)
PRESETS = {k: tuple(v / MM for v in p) for k, p in PRESETS_MM.items()}
WORKSPACE_STEP = WORKSPACE_STEP_MM / MM
WORKSPACE_RADIUS = WORKSPACE_RADIUS_MM / MM
CUSTOM = "Custom"
SEQ_DIR = HERE / "saved_sequences"

# =============================================================================
# ROBOT MATHS (Pinocchio + Pink)
# =============================================================================
model = pin.buildModelFromUrdf(URDF_PATH)

# Add a "tcp" frame at the gripper tip, so everything works in tip coordinates.
_hand_id = model.getFrameId(HAND_FRAME)
_hand = model.frames[_hand_id]
model.addFrame(pin.Frame(
    "tcp", _hand.parentJoint, _hand_id,
    _hand.placement * pin.SE3(np.eye(3), np.array([0.0, 0.0, TCP_OFFSET])),
    pin.FrameType.OP_FRAME,
))
TCP_ID = model.getFrameId("tcp")

data = model.createData()
data_dyn = model.createData()

q0 = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02])
configuration = pink.Configuration(model, data, q0)
task = FrameTask("tcp", position_cost=1.0, orientation_cost=0.5)

LOWER = model.lowerPositionLimit[:NUM_JOINTS]
UPPER = model.upperPositionLimit[:NUM_JOINTS]


def tip_position():
    return configuration.get_transform_frame_to_world("tcp").translation.copy()


HOME = tip_position()
PRESETS = {"Home": tuple(HOME), **PRESETS}

# =============================================================================
# VIEWER AND 3D SCENE
# =============================================================================
server = viser.ViserServer()
server.scene.set_up_direction("+z")

urdf = load_robot_description("panda_description")
viser_urdf = ViserUrdf(server, urdf_or_path=urdf)
viewer_joint_names = list(viser_urdf.get_actuated_joint_limits().keys())


def show_robot(q):
    cfg = [q[model.joints[model.getJointId(n)].idx_q] for n in viewer_joint_names]
    viser_urdf.update_cfg(np.array(cfg))


ground = server.scene.add_grid(
    "/ground", width=3.0, height=3.0, plane="xy",
    cell_size=0.1, section_size=0.5, plane_opacity=1.0,
    position=(0.0, 0.0, -0.001),
)
server.scene.add_frame("/origin", axes_length=0.12, axes_radius=0.004)

cube = server.scene.add_box(
    "/cube", color=(230, 150, 40),
    dimensions=(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
    position=(CUBE_HOME[0], CUBE_HOME[1], CUBE_SIZE / 2),
)
cube_label = server.scene.add_label(
    "/cube/label", f"{CUBE_MASS * 1000:.0f} g", position=(0.0, 0.0, 0.05),
    anchor="bottom-center",
)

target = server.scene.add_transform_controls(
    "/target", scale=0.15, position=tuple(HOME), disable_rotations=True,
)
target_ball = server.scene.add_icosphere(
    "/target_ball", radius=0.012, color=(60, 200, 90), opacity=0.7,
    position=tuple(HOME),
)
trail_points = deque(maxlen=150)
trail = None
workspace_cloud = None


def apply_scene_theme(dark):
    if dark:
        ground.plane_color = (45, 48, 55)
        ground.cell_color = (70, 74, 82)
        ground.section_color = (110, 115, 125)
    else:
        ground.plane_color = (228, 231, 235)
        ground.cell_color = (200, 204, 210)
        ground.section_color = (150, 155, 165)


apply_scene_theme(False)


def reset_camera(client):
    client.camera.position = (1.3, -1.1, 0.9)
    client.camera.look_at = (0.3, 0.0, 0.25)


@server.on_client_connect
def _(client):
    reset_camera(client)


def notify(title, body, color=None):
    for client in server.get_clients().values():
        client.add_notification(title, body, auto_close_seconds=4.0, color=color)


# =============================================================================
# SHARED STATE
# =============================================================================
state = {
    "mode": "Manual",          # Manual / Sequence
    "estop": False,
    "holding": False,
    "reach": "ok",             # ok / moving / out
    "stall_time": 0.0,
    "last_target": np.array(HOME),
    "warned_target": None,
    "prev_qd": np.zeros(model.nv),
    "stopping": False,
}
seq = {"steps": [], "index": 0, "started": False, "t0": 0.0,
       "arrived_at": None, "trial": 1, "trials_total": 1, "first_report": 0}
seq_nodes = []                 # 3D markers (ball, label) for the move steps
trials_summary = ""
hw = {"mode": "offline",       # offline / demo / serial
      "ser": None, "port": "", "measured": None, "measured_time": 0.0,
      "sent": 0, "received": 0, "last_rx": "", "error": ""}
cycle = {"active": False, "kind": "", "t0": 0.0, "samples": []}
reports = []

history = deque(maxlen=300)          # (t, simulated, measured) for the plot
compare_window = deque(maxlen=250)   # last 5 s of (simulated, measured)
log_rows = []                        # for CSV export
log_start = time.time()
preset_nodes = {}

# Fictitious "measured" currents: each servo gets its own gain error and offset.
_rng = np.random.default_rng(7)
FAKE_GAIN = 1.0 + _rng.uniform(-0.12, 0.15, NUM_JOINTS)
FAKE_OFFSET = _rng.uniform(0.03, 0.12, NUM_JOINTS)
FAKE_LAG = 0.08                      # s
fake_state = np.zeros(NUM_JOINTS)

# =============================================================================
# GUI: always-visible header
# =============================================================================
server.gui.configure_theme(
    brand_color=BRAND, control_width="large", show_share_button=False,
)
server.gui.add_markdown("### Pick-and-place arm simulator")
estop_btn = server.gui.add_button("EMERGENCY STOP", color="red",
                                  icon=viser.Icon.ALERT_OCTAGON)
estop_banner = server.gui.add_markdown("", visible=False)
estop_reset = server.gui.add_button("Reset emergency stop", color="yellow",
                                    icon=viser.Icon.RESTORE, visible=False)
mode_md = server.gui.add_markdown("")

tabs = server.gui.add_tab_group()

# ---------------- Control tab ----------------
with tabs.add_tab("Control", icon=viser.Icon.TARGET):
    with server.gui.add_folder("Status"):
        reach_md = server.gui.add_markdown("")
        tip_txt = server.gui.add_text("Tip position (mm)", initial_value="",
                                      disabled=True)

    with server.gui.add_folder("Move gripper tip"):
        x_in = server.gui.add_number("X (mm)", initial_value=round(HOME[0] * MM), step=1)
        y_in = server.gui.add_number("Y (mm)", initial_value=round(HOME[1] * MM), step=1)
        z_in = server.gui.add_number("Z (mm)", initial_value=round(HOME[2] * MM),
                                     step=1, min=0)
        go_btn = server.gui.add_button("Go to position", icon=viser.Icon.CURRENT_LOCATION)
        speed = server.gui.add_slider(
            "Speed (%)", min=10, max=100, step=5, initial_value=60,
            hint="Percentage of the max joint speed set in the Joints tab")

    with server.gui.add_folder(f"Cube ({CUBE_MASS * 1000:.0f} g)"):
        cube_md = server.gui.add_markdown("On the ground")
        grab_btn = server.gui.add_button("Grab cube", color="green",
                                         icon=viser.Icon.HAND_GRAB)
        release_btn = server.gui.add_button("Release cube", icon=viser.Icon.ARROW_DOWN)
        cube_x = server.gui.add_number("Cube X (mm)", initial_value=CUBE_HOME_MM[0], step=1)
        cube_y = server.gui.add_number("Cube Y (mm)", initial_value=CUBE_HOME_MM[1], step=1)
        place_btn = server.gui.add_button("Put cube here", icon=viser.Icon.CUBE)

# ---------------- Sequence tab ----------------
STEP_TYPES = ["Move to position", "Grab cube", "Release cube", "Wait 1 s"]
with tabs.add_tab("Sequence", icon=viser.Icon.LIST):
    with server.gui.add_folder("Add step"):
        step_type = server.gui.add_dropdown("Step", STEP_TYPES)
        step_preset = server.gui.add_dropdown("Position", list(PRESETS.keys()) + [CUSTOM])
        cx_in = server.gui.add_number("Custom X (mm)", initial_value=round(HOME[0] * MM),
                                      step=1, visible=False)
        cy_in = server.gui.add_number("Custom Y (mm)", initial_value=round(HOME[1] * MM),
                                      step=1, visible=False)
        cz_in = server.gui.add_number("Custom Z (mm)", initial_value=round(HOME[2] * MM),
                                      step=1, visible=False)
        copy_btn = server.gui.add_button("Copy from 3D handle", icon=viser.Icon.TARGET,
                                         visible=False)
        check_md = server.gui.add_markdown("", visible=False)
        step_add = server.gui.add_button("Add step", icon=viser.Icon.PLUS)
        step_undo = server.gui.add_button("Undo last step", icon=viser.Icon.ARROW_BACK_UP)
        step_clear = server.gui.add_button("Clear all", icon=viser.Icon.TRASH)
        step_demo = server.gui.add_button("Load pick-and-place demo",
                                          icon=viser.Icon.PLAYLIST_ADD)
        with server.gui.add_folder("Save custom point as preset", expand_by_default=False):
            preset_name = server.gui.add_text("Preset name", initial_value="My point")
            preset_save = server.gui.add_button("Save as preset",
                                                icon=viser.Icon.DEVICE_FLOPPY)

    with server.gui.add_folder("Run"):
        trials_in = server.gui.add_number("Number of trials", initial_value=1,
                                          min=1, max=100, step=1)
        reset_cube_cb = server.gui.add_checkbox(
            "Reset cube before each trial", initial_value=True,
            hint="Put the cube back at Cube X, Y (Control tab) before every trial")
        seq_run = server.gui.add_button("Run sequence", color="green",
                                        icon=viser.Icon.PLAYER_PLAY)
        seq_stop = server.gui.add_button(
            "Stop", icon=viser.Icon.PLAYER_STOP,
            hint="Stops any motion (sequence or manual) and holds position")
        seq_progress = server.gui.add_progress_bar(0.0, color=BRAND)
        trial_md = server.gui.add_markdown("")
        seq_md = server.gui.add_markdown("*No steps yet*")

    with server.gui.add_folder("Saved sequences"):
        server.gui.add_markdown("Saves the steps and your presets together.")
        seq_name = server.gui.add_text("Sequence name", initial_value="pick_and_place")
        seq_save_btn = server.gui.add_button("Save sequence", color="green",
                                             icon=viser.Icon.DEVICE_FLOPPY)
        seq_saved_dd = server.gui.add_dropdown("Saved sequences", ["(none)"])
        seq_refresh_btn = server.gui.add_button("Refresh list", icon=viser.Icon.REFRESH)
        seq_load_btn = server.gui.add_button("Load sequence", icon=viser.Icon.FOLDER_OPEN)
        seq_delete_btn = server.gui.add_button("Delete saved sequence",
                                               icon=viser.Icon.TRASH)

# ---------------- Joints tab ----------------
with tabs.add_tab("Joints", icon=viser.Icon.ROBOT):
    joints_md = server.gui.add_markdown("")
    with server.gui.add_folder("Speed and acceleration limits"):
        max_speed = server.gui.add_number("Max joint speed (rad/s)",
                                          initial_value=MAX_JOINT_SPEED, step=0.1, min=0.1)
        max_accel = server.gui.add_number("Max joint accel (rad/s²)",
                                          initial_value=MAX_JOINT_ACCEL, step=0.5, min=0.5)
        limits_md = server.gui.add_markdown("")

# ---------------- Currents tab ----------------
with tabs.add_tab("Currents", icon=viser.Icon.CHART_LINE):
    source_md = server.gui.add_markdown("")
    plot_mode = server.gui.add_dropdown(
        "Show", ["Both", "Simulated", "Measured", "Error (meas - sim)"])
    plot_joint = server.gui.add_dropdown(
        "Joint", ["All"] + [f"J{j + 1}" for j in range(NUM_JOINTS)])
    plot = server.gui.add_plotly(figure=go.Figure(), aspect=1.3)
    server.gui.add_markdown("*Solid = simulated, dotted = measured.*")
    currents_md = server.gui.add_markdown("Waiting for data...")
    with server.gui.add_folder("Data log"):
        recording = server.gui.add_checkbox("Recording", initial_value=True)
        log_txt = server.gui.add_text("Samples", initial_value="0", disabled=True)
        server.gui.add_markdown("*Export is in the Report tab.*")
        clear_log_btn = server.gui.add_button("Clear log", icon=viser.Icon.TRASH)

# ---------------- Report tab ----------------
with tabs.add_tab("Report", icon=viser.Icon.REPORT_ANALYTICS):
    server.gui.add_markdown("A report is made after each sequence trial "
                            "(or when a run is stopped).")
    report_md = server.gui.add_markdown("*No cycles run yet*")
    with server.gui.add_folder("Export"):
        server.gui.add_markdown(
            "One Excel file with three sheets: **Summary**, **Cycle reports** "
            "and **Data log** (every sample from the Currents tab).")
        export_all_btn = server.gui.add_button("Export all data", color="green",
                                               icon=viser.Icon.FILE_SPREADSHEET)
    with server.gui.add_folder("All cycles this session"):
        report_hist_md = server.gui.add_markdown("*None yet*")
        report_clear = server.gui.add_button("Clear reports", icon=viser.Icon.TRASH)

# ---------------- Hardware tab ----------------
with tabs.add_tab("Hardware", icon=viser.Icon.CPU):
    hw_status_md = server.gui.add_markdown("")
    with server.gui.add_folder("Connection"):
        port_dd = server.gui.add_dropdown("Port", [DEMO_PORT])
        baud_dd = server.gui.add_dropdown("Baud rate", BAUD_RATES)
        port_refresh = server.gui.add_button("Refresh ports", icon=viser.Icon.REFRESH)
        connect_btn = server.gui.add_button("Connect", color="green",
                                            icon=viser.Icon.PLUG_CONNECTED)
        disconnect_btn = server.gui.add_button("Disconnect",
                                               icon=viser.Icon.PLUG_CONNECTED_X)
        mirror_cb = server.gui.add_checkbox(
            "Mirror motion to arm", initial_value=False,
            hint="Send the simulated joint angles to the ESP 50 times a second")
        traffic_md = server.gui.add_markdown("")
    with server.gui.add_folder("Message format (for the ESP firmware)",
                               expand_by_default=False):
        server.gui.add_markdown(
            "One text line per message, ending in a newline.\n\n"
            "**PC to ESP**\n"
            f"- `J,a1,...,a{NUM_JOINTS}` joint angles in degrees (50 Hz while mirroring)\n"
            "- `G,1` / `G,0` close / open gripper\n"
            "- `S` emergency stop: hold still and ignore `J` until `R`\n"
            "- `R` reset after an emergency stop\n\n"
            "**ESP to PC**\n"
            f"- `I,i1,...,i{NUM_JOINTS}` servo currents in amps (e.g. 20-50 Hz)\n"
            "- any other line is shown here as the last message\n\n"
            "Example: `J,0.0,-45.0,0.0,-135.0,0.0,90.0,45.0`"
        )

# ---------------- Setup tab ----------------
with tabs.add_tab("Setup", icon=viser.Icon.SETTINGS):
    with server.gui.add_folder("Display"):
        dark_cb = server.gui.add_checkbox("Dark mode", initial_value=False)
        show_trail = server.gui.add_checkbox("Show tip trail", initial_value=True)
        show_presets = server.gui.add_checkbox("Show preset markers", initial_value=True)
        show_labels = server.gui.add_checkbox("Show step numbers", initial_value=True)
        show_grid = server.gui.add_checkbox("Show ground", initial_value=True)
        cam_btn = server.gui.add_button("Reset camera", icon=viser.Icon.CAMERA)
    with server.gui.add_folder("Workspace map"):
        ws_height = server.gui.add_number("Scan height Z (mm)",
                                          initial_value=CUBE_SIZE_MM / 2, step=5, min=0)
        ws_scan = server.gui.add_button("Scan workspace", color="green",
                                        icon=viser.Icon.RADAR)
        ws_clear = server.gui.add_button("Clear map", icon=viser.Icon.TRASH)
        ws_progress = server.gui.add_progress_bar(0.0, color=BRAND)
        ws_md = server.gui.add_markdown(
            "🟩 gripper can point straight down  \n"
            "🟨 reachable, but not pointing down  \n"
            "(no colour) out of reach")


# =============================================================================
# HELPERS
# =============================================================================
def set_fields(p):
    """Show a position (given in metres) in the Control tab fields, in mm."""
    x_in.value, y_in.value, z_in.value = (round(float(p[0]) * MM, 1),
                                          round(float(p[1]) * MM, 1),
                                          round(float(p[2]) * MM, 1))


def set_target(p):
    """Move the target. Ignored while the emergency stop is active."""
    if state["estop"]:
        return
    state["stopping"] = False
    p = np.array(p, dtype=float)
    p[2] = max(p[2], 0.0)
    target.position = tuple(p)
    set_fields(p)


def begin_stop():
    """Brake smoothly (within the acceleration limit), then hold that pose
    until the next command."""
    if not state["estop"]:
        state["stopping"] = True


def set_mode(mode):
    state["mode"] = mode
    icons = {"Manual": "🕹️", "Sequence": "📋"}
    mode_md.content = f"**Mode:** {icons[mode]} {mode}"


set_mode("Manual")


def blocked():
    """True (with a message) if the E-stop is active."""
    if state["estop"]:
        notify("Emergency stop active", "Reset the emergency stop first",
               color="red")
        return True
    return False


@target.on_update
def _(_):
    if state["estop"]:
        target.position = tuple(tip_position())  # snap back: arm is frozen
        return
    state["stopping"] = False
    set_fields(target.position)
    if state["mode"] != "Manual":
        stop_everything("Stopped: target moved by hand")


def stop_everything(message="Stopped"):
    was = state["mode"]
    set_mode("Manual")
    if was == "Sequence":
        seq_md.content = render_sequence(message)
    if cycle["active"]:
        finish_cycle(completed=False, note=message)


def do_grab():
    dist = np.linalg.norm(tip_position() - np.array(cube.position))
    if dist < 0.04:
        state["holding"] = True
        cube_md.content = "✋ **Held by gripper**"
        cube_label.text = f"{CUBE_MASS * 1000:.0f} g (held)"
        hw_send("G,1")
        return True
    cube_md.content = f"❌ Too far to grab ({dist * MM:.0f} mm away)"
    return False


def do_release():
    state["holding"] = False
    p = np.array(cube.position)
    cube.position = (float(p[0]), float(p[1]), CUBE_SIZE / 2)
    cube_md.content = "On the ground"
    cube_label.text = f"{CUBE_MASS * 1000:.0f} g"
    hw_send("G,0")


def refresh_preset_dropdowns():
    step_preset.options = list(PRESETS.keys()) + [CUSTOM]


def draw_presets():
    for ball, _label in preset_nodes.values():
        ball.remove()  # removing the ball also removes its label
    preset_nodes.clear()
    for i, (name, p) in enumerate(PRESETS.items()):
        ball = server.scene.add_icosphere(
            f"/presets/p{i}", radius=0.01, color=(70, 130, 230), opacity=0.6,
            position=tuple(p), visible=show_presets.value,
        )
        label = server.scene.add_label(
            f"/presets/p{i}/label", name, position=(0.0, 0.0, 0.02),
            anchor="bottom-center", visible=show_presets.value,
        )
        preset_nodes[name] = (ball, label)


draw_presets()


# =============================================================================
# EMERGENCY STOP
# =============================================================================
@estop_btn.on_click
def _(_):
    trigger_estop()


def trigger_estop():
    if state["estop"]:
        return
    stop_everything("🛑 Emergency stop")
    state["estop"] = True
    state["stopping"] = False
    state["prev_qd"] = np.zeros(model.nv)
    target.position = tuple(tip_position())
    hw_send("S")
    estop_btn.visible = False
    estop_banner.content = ("## 🛑 EMERGENCY STOP ACTIVE\n"
                            "The arm is frozen and commands are blocked.")
    estop_banner.visible = True
    estop_reset.visible = True
    notify("EMERGENCY STOP", "All motion stopped", color="red")


@estop_reset.on_click
def _(_):
    state["estop"] = False
    set_fields(target.position)
    hw_send("R")
    estop_banner.visible = False
    estop_reset.visible = False
    estop_btn.visible = True
    notify("Emergency stop reset", "Motion commands are allowed again",
           color="green")


# =============================================================================
# CONTROL TAB ACTIONS
# =============================================================================
@go_btn.on_click
def _(_):
    if blocked():
        return
    p = np.array([x_in.value, y_in.value, z_in.value]) / MM
    if not validate_point(p, show_in_sequence_tab=False):
        return
    stop_everything()
    set_target(p)


@preset_save.on_click
def _(_):
    p = custom_point()
    if not validate_point(p):
        return
    name = preset_name.value.strip() or f"Point {len(PRESETS)}"
    if name == CUSTOM:
        name = "Custom point"
    PRESETS[name] = tuple(float(v) for v in p)
    refresh_preset_dropdowns()
    draw_presets()
    step_preset.value = name
    notify("Preset saved", f"'{name}' added to the list", color="green")


@grab_btn.on_click
def _(_):
    if not blocked():
        do_grab()


@release_btn.on_click
def _(_):
    if not blocked():
        do_release()


@place_btn.on_click
def _(_):
    if not state["holding"]:
        cube.position = (cube_x.value / MM, cube_y.value / MM, CUBE_SIZE / 2)


# =============================================================================
# SEQUENCE TAB
# =============================================================================
def describe(step):
    if step["kind"] == "move":
        return f"Move to **{step['name']}**"
    if step["kind"] == "grab":
        return "Grab cube"
    if step["kind"] == "release":
        return "Release cube"
    return f"Wait {step['secs']:.0f} s"


def render_sequence(note=""):
    steps = seq["steps"]
    if not steps:
        return "*No steps yet*"
    running = state["mode"] == "Sequence"
    lines = []
    for i, s in enumerate(steps):
        if running and i == seq["index"]:
            mark = "▶️"
        elif running and i < seq["index"]:
            mark = "✅"
        else:
            mark = f"{i + 1}."
        lines.append(f"{mark} {describe(s)}  ")
    if note:
        lines.append(f"\n{note}")
    return "\n".join(lines)


def draw_sequence():
    """Numbered markers in 3D for every move step, joined by a path line."""
    for ball, _label in seq_nodes:
        ball.remove()  # removing the ball also removes its label
    seq_nodes.clear()
    points = []
    for i, s in enumerate(seq["steps"]):
        if s["kind"] != "move":
            continue
        points.append(s["pos"])
        ball = server.scene.add_icosphere(
            f"/seq_points/s{i}", radius=0.012, color=(255, 80, 80),
            position=tuple(s["pos"]),
        )
        label = server.scene.add_label(
            f"/seq_points/s{i}/label", str(i + 1), position=(0.0, 0.0, 0.02),
            anchor="bottom-center", visible=show_labels.value,
        )
        seq_nodes.append((ball, label))
    if len(points) < 2:
        server.scene.add_line_segments(
            "/seq_path", points=np.zeros((0, 2, 3)), colors=(255, 80, 80),
            visible=False,
        )
        return
    segs = np.array([[points[i], points[i + 1]] for i in range(len(points) - 1)])
    server.scene.add_line_segments(
        "/seq_path", points=segs, colors=(255, 80, 80),
        thickness=2.0, thickness_units="screen",
    )


def refresh_sequence(note=""):
    seq_md.content = render_sequence(note)
    draw_sequence()


def custom_point():
    """The Custom X, Y, Z fields, converted to metres."""
    return np.array([cx_in.value, cy_in.value, cz_in.value], dtype=float) / MM


def fmt_mm(p):
    return f"({p[0] * MM:.0f}, {p[1] * MM:.0f}, {p[2] * MM:.0f}) mm"


def update_step_fields(_=None):
    is_move = step_type.value == "Move to position"
    custom = is_move and step_preset.value == CUSTOM
    step_preset.visible = is_move
    for h in (cx_in, cy_in, cz_in, copy_btn):
        h.visible = custom
    if not custom:
        check_md.visible = False


step_type.on_update(update_step_fields)
step_preset.on_update(update_step_fields)


@copy_btn.on_click
def _(_):
    p = np.array(target.position) * MM
    cx_in.value, cy_in.value, cz_in.value = (round(float(p[0]), 1),
                                             round(float(p[1]), 1),
                                             round(float(max(p[2], 0.0)), 1))


def validate_point(p, show_in_sequence_tab=True):
    """Check a point (metres) before using it. Shows an error if it is bad."""
    ok, message = check_point(p)
    if show_in_sequence_tab:
        check_md.content = message
        check_md.visible = bool(message)
    if not ok:
        notify("Position out of reach", f"{fmt_mm(p)}: {message[2:]}", color="red")
    elif message:
        notify("Position check", f"{fmt_mm(p)}: {message[2:]}", color="yellow")
    return ok


def make_step(kind_label, preset):
    if kind_label == "Move to position":
        if preset == CUSTOM:
            p = custom_point()
            return {"kind": "move", "name": fmt_mm(p), "pos": p}
        return {"kind": "move", "name": preset, "pos": np.array(PRESETS[preset])}
    if kind_label == "Grab cube":
        return {"kind": "grab"}
    if kind_label == "Release cube":
        return {"kind": "release"}
    return {"kind": "wait", "secs": 1.0}


def editing_allowed():
    if state["mode"] == "Sequence":
        notify("Sequence running", "Stop it before editing", color="yellow")
        return False
    return True


@step_add.on_click
def _(_):
    if not editing_allowed():
        return
    new_step = make_step(step_type.value, step_preset.value)
    if new_step["kind"] == "move" and not validate_point(new_step["pos"]):
        return
    seq["steps"].append(new_step)
    refresh_sequence()


@step_undo.on_click
def _(_):
    if seq["steps"] and editing_allowed():
        seq["steps"].pop()
        refresh_sequence()


@step_clear.on_click
def _(_):
    if editing_allowed():
        seq["steps"].clear()
        seq_progress.value = 0.0
        trial_md.content = ""
        refresh_sequence()


@step_demo.on_click
def _(_):
    if not editing_allowed():
        return
    plan = ["Home", "Above pick", "Pick", "GRAB", "Above pick",
            "Above place", "Place", "RELEASE", "Above place", "Home"]
    seq["steps"] = []
    for p in plan:
        if p == "GRAB":
            seq["steps"].append({"kind": "grab"})
        elif p == "RELEASE":
            seq["steps"].append({"kind": "release"})
        elif p in PRESETS:
            seq["steps"].append(make_step("Move to position", p))
    refresh_sequence()


def start_step(i):
    seq["index"] = i
    seq["started"] = False
    seq["arrived_at"] = None
    n_steps = len(seq["steps"])
    done_steps = (seq["trial"] - 1) * n_steps + i
    seq_progress.value = 100.0 * done_steps / (seq["trials_total"] * n_steps)
    seq_md.content = render_sequence()


def start_trial():
    k, n = seq["trial"], seq["trials_total"]
    if reset_cube_cb.value and not state["holding"]:
        cube.position = (cube_x.value / MM, cube_y.value / MM, CUBE_SIZE / 2)
    start_cycle(f"Trial {k}/{n}" if n > 1 else "Sequence")
    trial_md.content = f"**Trial {k} of {n}**" if n > 1 else ""
    start_step(0)


@seq_run.on_click
def _(_):
    if blocked():
        return
    if not seq["steps"]:
        notify("Nothing to run", "Add some steps first", color="yellow")
        return
    stop_everything()
    seq["trial"] = 1
    seq["trials_total"] = max(1, int(trials_in.value))
    seq["first_report"] = len(reports)
    set_mode("Sequence")
    start_trial()


@seq_stop.on_click
def _(_):
    stop_everything("⏹️ Stopped by user")
    begin_stop()


def abort_sequence(reason):
    if seq["trials_total"] > 1:
        reason += f" (trial {seq['trial']} of {seq['trials_total']})"
    stop_everything(f"❌ {reason}")
    begin_stop()
    notify("Sequence stopped", reason, color="red")


def summarise_trials():
    """Mean and spread over the trials of the run that just finished."""
    global trials_summary
    runs = [r for r in reports[seq["first_report"]:] if r["result"] == "Completed"]
    if len(runs) < 2:
        trials_summary = ""
        return
    t = np.array([r["duration_s"] for r in runs])
    peak = np.array([r["peak_total_sim"] for r in runs])
    energy = np.array([r["energy_sim_J"] for r in runs])
    trials_summary = (
        f"**Last run: {len(runs)} trials**  \n"
        f"Cycle time: {t.mean():.1f} ± {t.std(ddof=1):.1f} s  \n"
        f"Peak total current: {peak.mean():.2f} ± {peak.std(ddof=1):.2f} A  \n"
        f"Energy: {energy.mean():.0f} ± {energy.std(ddof=1):.0f} J"
    )
    report_hist_md.content = render_report_history()


def update_sequence(now):
    steps = seq["steps"]
    i = seq["index"]
    s = steps[i]

    if not seq["started"]:
        seq["started"] = True
        seq["t0"] = now
        if s["kind"] == "move":
            set_target(s["pos"])
            return
        if s["kind"] == "grab" and not do_grab():
            abort_sequence(f"Step {i + 1}: cube is not at the gripper")
            return
        if s["kind"] == "release":
            do_release()

    done = False
    if s["kind"] == "move":
        if state["reach"] == "ok":
            if seq["arrived_at"] is None:
                seq["arrived_at"] = now
            done = now - seq["arrived_at"] > 0.3
        elif state["reach"] == "out":
            abort_sequence(f"Step {i + 1}: '{s['name']}' is out of reach")
            return
        elif now - seq["t0"] > 15.0:
            abort_sequence(f"Step {i + 1}: timed out")
            return
    elif s["kind"] == "wait":
        done = now - seq["t0"] > s["secs"]
    else:
        done = now - seq["t0"] > 0.4   # short pause after grab/release

    if not done:
        return
    if i + 1 < len(steps):
        start_step(i + 1)
        return

    finish_cycle(completed=True)
    if seq["trial"] < seq["trials_total"]:
        seq["trial"] += 1
        start_trial()
        return

    n = seq["trials_total"]
    seq_progress.value = 100.0
    summarise_trials()
    stop_everything("🏁 **Sequence finished**" + (f" ({n} trials)" if n > 1 else ""))
    notify("Sequence finished",
           (f"{n} trials completed. " if n > 1 else f"{len(steps)} steps completed. ")
           + "See the Report tab.", color="green")


# ---------------- Saved sequences ----------------
def list_sequences():
    if not SEQ_DIR.exists():
        return ["(none)"]
    names = sorted(p.stem for p in SEQ_DIR.glob("*.json"))
    return names or ["(none)"]


def clean_name(text, default):
    return "".join(c for c in text.strip() if c.isalnum() or c in "-_ ") or default


@seq_save_btn.on_click
def _(_):
    if not seq["steps"]:
        notify("Nothing to save", "The sequence is empty", color="yellow")
        return
    name = clean_name(seq_name.value, "sequence")
    SEQ_DIR.mkdir(exist_ok=True)
    path = SEQ_DIR / f"{name}.json"
    content = {
        "version": 2,
        "units": "mm",
        "saved": datetime.now().isoformat(timespec="seconds"),
        "presets_mm": {k: [round(v * MM, 1) for v in p] for k, p in PRESETS.items()
                       if k != "Home"},
        "steps": [step_to_json(s) for s in seq["steps"]],
    }
    path.write_text(json.dumps(content, indent=2), encoding="utf-8")
    seq_saved_dd.options = list_sequences()
    seq_saved_dd.value = name
    notify("Sequence saved", f"'{name}' ({len(seq['steps'])} steps)", color="green")


@seq_refresh_btn.on_click
def _(_):
    seq_saved_dd.options = list_sequences()


@seq_load_btn.on_click
def _(_):
    if not editing_allowed():
        return
    name = seq_saved_dd.value
    path = SEQ_DIR / f"{name}.json"
    if name == "(none)" or not path.exists():
        notify("Nothing to load", "Save a sequence first", color="yellow")
        return
    try:
        content = json.loads(path.read_text(encoding="utf-8"))
        steps = [step_from_json(d) for d in content["steps"]]
        presets = {k: tuple(v / MM for v in p)
                   for k, p in content.get("presets_mm", {}).items()}
    except Exception as exc:
        notify("Could not load sequence", f"{path.name}: {exc}", color="red")
        return
    if presets:
        PRESETS.update(presets)   # adds the file's presets to your current list
        refresh_preset_dropdowns()
        draw_presets()
    seq["steps"] = steps
    seq_progress.value = 0.0
    trial_md.content = ""
    seq_name.value = name
    refresh_sequence()
    notify("Sequence loaded", f"'{name}' ({len(steps)} steps)", color="green")


@seq_delete_btn.on_click
def _(_):
    name = seq_saved_dd.value
    path = SEQ_DIR / f"{name}.json"
    if name == "(none)" or not path.exists():
        return
    path.unlink()
    seq_saved_dd.options = list_sequences()
    notify("Sequence deleted", f"'{name}' removed")


seq_saved_dd.options = list_sequences()


# =============================================================================
# HARDWARE CONNECTION
# =============================================================================
def list_ports():
    ports = []
    if HAVE_SERIAL:
        ports = [p.device for p in serial.tools.list_ports.comports()]
    return [DEMO_PORT] + ports


def hw_send(line):
    """Send one line to the ESP (does nothing unless connected to a real port)."""
    ser = hw["ser"]
    if hw["mode"] != "serial" or ser is None:
        return
    try:
        ser.write((line + "\n").encode("ascii"))
        hw["sent"] += 1
    except Exception as exc:  # cable pulled, port closed, ...
        hw_disconnect(error=f"Write failed: {exc}")


def hw_reader(ser):
    """Background thread: read lines from the ESP."""
    while hw["ser"] is ser:
        try:
            raw = ser.readline()
        except Exception as exc:
            if hw["ser"] is ser:
                hw_disconnect(error=f"Read failed: {exc}")
            return
        if not raw:
            continue
        line = raw.decode("ascii", errors="ignore").strip()
        hw["received"] += 1
        hw["last_rx"] = line[:80]
        if line.startswith("I,"):
            try:
                values = np.array([float(v) for v in line[2:].split(",")])
            except ValueError:
                continue
            if len(values) == NUM_JOINTS:
                hw["measured"] = values
                hw["measured_time"] = time.time()


def hw_connect(port, baud):
    hw_disconnect()
    hw["sent"] = hw["received"] = 0
    hw["last_rx"] = hw["error"] = ""
    if port == DEMO_PORT:
        hw["mode"] = "demo"
        hw["port"] = port
        return True
    if not HAVE_SERIAL:
        hw["error"] = "pyserial is not installed (run: pip install pyserial)"
        return False
    try:
        ser = serial.serial_for_url(port, baudrate=baud, timeout=0.1)
    except Exception as exc:
        hw["error"] = f"Could not open {port}: {exc}"
        return False
    hw["ser"] = ser
    hw["mode"] = "serial"
    hw["port"] = port
    threading.Thread(target=hw_reader, args=(ser,), daemon=True).start()
    if state["estop"]:
        hw_send("S")
    return True


def hw_disconnect(error=""):
    ser = hw["ser"]
    hw["ser"] = None
    hw["mode"] = "offline"
    hw["measured"] = None
    if error:
        hw["error"] = error
        notify("Hardware disconnected", error, color="red")
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


@port_refresh.on_click
def _(_):
    port_dd.options = list_ports()


@connect_btn.on_click
def _(_):
    if hw_connect(port_dd.value, int(baud_dd.value)):
        notify("Connected", hw["port"], color="green")
    else:
        notify("Connection failed", hw["error"], color="red")


@disconnect_btn.on_click
def _(_):
    hw_disconnect()
    mirror_cb.value = False


def measured_is_live():
    return (hw["mode"] == "serial" and hw["measured"] is not None
            and time.time() - hw["measured_time"] < MEASURED_TIMEOUT)


def render_hw_status():
    if hw["mode"] == "serial":
        light = f"🟢 **Connected** to `{hw['port']}`"
    elif hw["mode"] == "demo":
        light = "🟡 **Demo connection** (no real ESP)"
    else:
        light = "⚪ **Not connected**"
    if hw["error"]:
        light += f"  \n❌ {hw['error']}"
    if not HAVE_SERIAL:
        light += "  \n*pyserial not installed: only the demo connection is available.*"
    return light


def render_traffic():
    mirror = "on" if (mirror_cb.value and hw["mode"] != "offline") else "off"
    last = hw["last_rx"] or "-"
    return (f"Mirroring: **{mirror}**  \n"
            f"Sent: {hw['sent']}  |  Received: {hw['received']}  \n"
            f"Last message: `{last}`")


port_dd.options = list_ports()


# =============================================================================
# CURRENTS: SIMULATED, MEASURED, COMPARISON
# =============================================================================
def simulated_currents(q, qd):
    tau = pin.computeGeneralizedGravity(model, data_dyn, q).copy()
    if state["holding"]:
        pin.computeJointJacobians(model, data_dyn, q)
        pin.updateFramePlacements(model, data_dyn)
        J = pin.getFrameJacobian(model, data_dyn, TCP_ID,
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        tau += J[:3].T @ np.array([0.0, 0.0, CUBE_MASS * 9.81])
    tau = tau[:NUM_JOINTS]
    currents = IDLE_CURRENT + np.abs(tau) / KT + 0.25 * np.abs(qd[:NUM_JOINTS])
    return tau, currents


def fictitious_measured(sim):
    """Pretend sensor: gain error, offset, first-order lag, and noise."""
    global fake_state
    alpha = DT / (FAKE_LAG + DT)
    fake_state = fake_state + alpha * (sim * FAKE_GAIN + FAKE_OFFSET - fake_state)
    noisy = fake_state + 0.04 * np.random.randn(NUM_JOINTS)
    return np.clip(noisy, 0.0, None)


def make_figure(dark):
    fig = go.Figure()
    if history:
        ts = np.array([h[0] for h in history])
        ts = ts - ts[-1]
        sim = np.array([h[1] for h in history])
        meas = np.array([h[2] for h in history])
        show = plot_mode.value
        joints = (range(NUM_JOINTS) if plot_joint.value == "All"
                  else [int(plot_joint.value[1:]) - 1])
        for j in joints:
            colour = JOINT_COLOURS[j]
            if show.startswith("Error"):
                fig.add_trace(go.Scatter(x=ts, y=meas[:, j] - sim[:, j],
                                         mode="lines", name=f"J{j + 1}",
                                         line=dict(color=colour, width=2)))
                continue
            if show in ("Both", "Simulated"):
                fig.add_trace(go.Scatter(x=ts, y=sim[:, j], mode="lines",
                                         name=f"J{j + 1} sim", legendgroup=f"J{j}",
                                         line=dict(color=colour, width=2)))
            if show in ("Both", "Measured"):
                fig.add_trace(go.Scatter(x=ts, y=meas[:, j], mode="lines",
                                         name=f"J{j + 1} meas", legendgroup=f"J{j}",
                                         line=dict(color=colour, width=1.5, dash="dot")))
    if plot_mode.value.startswith("Error"):
        y_title, y_range = "Error (A)", [-1.0, 1.0]
    else:
        y_title, y_range = "Current (A)", [0, CURRENT_LIMIT * 1.2]
        fig.add_hline(y=CURRENT_LIMIT, line_dash="dash", line_color="red",
                      annotation_text="limit", annotation_position="top left")
    fig.update_layout(
        template="plotly_dark" if dark else "plotly_white",
        paper_bgcolor="rgba(0,0,0,0)",
        margin=dict(l=45, r=10, t=10, b=40),
        xaxis_title="Time (s)", yaxis_title=y_title, yaxis_range=y_range,
        legend=dict(orientation="h", y=-0.25),
    )
    return fig


def render_source():
    if measured_is_live():
        return "**Measured data:** 🟢 live from the ESP"
    return ("**Measured data:** 🟡 fictitious "
            "(no current readings from an ESP yet)")


def render_currents(sim, meas, tau):
    window = np.array(compare_window) if compare_window else None
    rows = ["| Joint | Torque | Simulated | Measured | RMS error (5 s) | |",
            "|---|---|---|---|---|---|"]
    for j in range(NUM_JOINTS):
        rms = (np.sqrt(np.mean((window[:, 1, j] - window[:, 0, j]) ** 2))
               if window is not None else 0.0)
        flag = ("🔴" if max(sim[j], meas[j]) > CURRENT_LIMIT
                else "🟡" if max(sim[j], meas[j]) > 0.8 * CURRENT_LIMIT else "🟢")
        rows.append(f"| J{j + 1} | {tau[j]:+.2f} Nm | {sim[j]:.2f} A | "
                    f"{meas[j]:.2f} A | {rms:.3f} A | {flag} |")
    rows.append("")
    rows.append(f"**Total supply current:** {sim.sum():.2f} A simulated, "
                f"{meas.sum():.2f} A measured  ")
    if window is not None:
        sim_total = window[:, 0].sum(axis=1).mean()
        meas_total = window[:, 1].sum(axis=1).mean()
        pct = 100.0 * (meas_total - sim_total) / max(sim_total, 1e-6)
        rows.append(f"**Model error (5 s average of totals):** {pct:+.1f} %")
    return "\n".join(rows)


CSV_HEADER = (["time_s", "mode", "estop", "holding_cube",
               "target_x_mm", "target_y_mm", "target_z_mm",
               "tip_x_mm", "tip_y_mm", "tip_z_mm"]
              + [f"q{j + 1}_deg" for j in range(NUM_JOINTS)]
              + [f"tau{j + 1}_Nm" for j in range(NUM_JOINTS)]
              + [f"I{j + 1}_sim_A" for j in range(NUM_JOINTS)]
              + [f"I{j + 1}_meas_A" for j in range(NUM_JOINTS)]
              + ["I_total_sim_A", "I_total_meas_A", "measured_source"])


def send_to_browser(event, path):
    if event.client is not None:
        event.client.send_file_download(path.name, path.read_bytes())


@clear_log_btn.on_click
def _(_):
    global log_start
    log_rows.clear()
    history.clear()
    compare_window.clear()
    log_start = time.time()
    log_txt.value = "0"


# =============================================================================
# CYCLE REPORT
# =============================================================================
def start_cycle(kind):
    cycle.update(active=True, kind=kind, t0=time.time(), samples=[])


def finish_cycle(completed, note=""):
    if not cycle["active"]:
        return
    cycle["active"] = False
    samples = cycle["samples"]
    if len(samples) < 2:
        return
    t = np.array([s[0] for s in samples])
    sim = np.array([s[1] for s in samples])
    meas = np.array([s[2] for s in samples])
    qd = np.array([s[3] for s in samples])
    held = np.array([s[4] for s in samples])
    duration = t[-1] - t[0]
    dt = np.diff(t, prepend=t[0])
    total_sim = sim.sum(axis=1)
    total_meas = meas.sum(axis=1)

    report = {
        "number": len(reports) + 1,
        "kind": cycle["kind"],
        "result": "Completed" if completed else "Stopped",
        "note": note,
        "time": datetime.now().strftime("%H:%M:%S"),
        "duration_s": duration,
        "held_s": float((held * dt).sum()),
        "peak_sim": sim.max(axis=0),
        "peak_meas": meas.max(axis=0),
        "rms_sim": np.sqrt((sim ** 2).mean(axis=0)),
        "max_speed": np.abs(qd).max(axis=0),
        "peak_total_sim": float(total_sim.max()),
        "peak_total_meas": float(total_meas.max()),
        "avg_total_sim": float(total_sim.mean()),
        "energy_sim_J": float(SUPPLY_VOLTAGE * (total_sim * dt).sum()),
        "energy_meas_J": float(SUPPLY_VOLTAGE * (total_meas * dt).sum()),
        "charge_mAh": float((total_sim * dt).sum() / 3.6),
        "error_pct": float(100.0 * (total_meas.mean() - total_sim.mean())
                           / max(total_sim.mean(), 1e-6)),
        "measured_source": "ESP" if measured_is_live() else "fictitious",
    }
    reports.append(report)
    report_md.content = render_report(report)
    report_hist_md.content = render_report_history()


def render_report(r):
    busiest = int(np.argmax(r["rms_sim"]))
    lines = [
        f"#### Cycle {r['number']}: {r['kind']} ({r['result'].lower()})",
        f"Finished at {r['time']}  ",
        f"**Cycle time:** {r['duration_s']:.1f} s  ",
        f"**Cube carried for:** {r['held_s']:.1f} s  ",
        f"**Peak total current:** {r['peak_total_sim']:.2f} A simulated, "
        f"{r['peak_total_meas']:.2f} A measured  ",
        f"**Average total current:** {r['avg_total_sim']:.2f} A  ",
        f"**Energy used ({SUPPLY_VOLTAGE:.0f} V supply):** {r['energy_sim_J']:.0f} J "
        f"simulated, {r['energy_meas_J']:.0f} J measured  ",
        f"**Charge used:** {r['charge_mAh']:.1f} mAh  ",
        f"**Measured vs simulated:** {r['error_pct']:+.1f} % "
        f"(measured data: {r['measured_source']})  ",
        f"**Busiest joint:** J{busiest + 1}",
        "",
        "| Joint | Peak sim | Peak meas | RMS sim | Max speed |",
        "|---|---|---|---|---|",
    ]
    for j in range(NUM_JOINTS):
        lines.append(f"| J{j + 1} | {r['peak_sim'][j]:.2f} A | "
                     f"{r['peak_meas'][j]:.2f} A | {r['rms_sim'][j]:.2f} A | "
                     f"{r['max_speed'][j]:.2f} rad/s |")
    if r["note"]:
        lines.append(f"\n{r['note']}")
    return "\n".join(lines)


def render_report_history():
    if not reports:
        return "*None yet*"
    lines = ["| # | Type | Result | Time | Peak total | Energy |",
             "|---|---|---|---|---|---|"]
    for r in reports:
        lines.append(f"| {r['number']} | {r['kind']} | {r['result']} | "
                     f"{r['duration_s']:.1f} s | {r['peak_total_sim']:.2f} A | "
                     f"{r['energy_sim_J']:.0f} J |")
    if trials_summary:
        lines.append("\n" + trials_summary)
    return "\n".join(lines)


REPORT_COLUMNS = (
    ["Cycle", "Type", "Result", "Finished", "Cycle time (s)", "Cube carried (s)",
     "Peak total sim (A)", "Peak total meas (A)", "Avg total sim (A)",
     "Energy sim (J)", "Energy meas (J)", "Charge (mAh)", "Meas vs sim (%)",
     "Measured data"]
    + [f"J{j + 1} peak sim (A)" for j in range(NUM_JOINTS)]
    + [f"J{j + 1} peak meas (A)" for j in range(NUM_JOINTS)]
    + [f"J{j + 1} RMS sim (A)" for j in range(NUM_JOINTS)]
    + [f"J{j + 1} max speed (rad/s)" for j in range(NUM_JOINTS)]
)


def report_row(r):
    return ([r["number"], r["kind"], r["result"], r["time"],
             round(r["duration_s"], 2), round(r["held_s"], 2),
             round(r["peak_total_sim"], 3), round(r["peak_total_meas"], 3),
             round(r["avg_total_sim"], 3), round(r["energy_sim_J"], 1),
             round(r["energy_meas_J"], 1), round(r["charge_mAh"], 2),
             round(r["error_pct"], 1), r["measured_source"]]
            + [round(float(v), 3) for v in r["peak_sim"]]
            + [round(float(v), 3) for v in r["peak_meas"]]
            + [round(float(v), 3) for v in r["rms_sim"]]
            + [round(float(v), 3) for v in r["max_speed"]])


def write_sheet(ws_, header, rows, widths=14):
    from openpyxl.styles import Font, PatternFill
    ws_.append(header)
    for cell in ws_[1]:
        cell.font = Font(bold=True, color="FFFFFF")
        cell.fill = PatternFill("solid", fgColor="E67828")
    for row in rows:
        ws_.append(row)
    ws_.freeze_panes = "A2"
    for i in range(1, len(header) + 1):
        ws_.column_dimensions[ws_.cell(1, i).column_letter].width = widths


@export_all_btn.on_click
def _(event):
    try:
        from openpyxl import Workbook
    except ImportError:
        notify("Excel export needs openpyxl",
               "In the Miniforge Prompt run: pip install openpyxl", color="red")
        return
    if not log_rows and not reports:
        notify("Nothing to export", "Run the arm or a sequence first", color="yellow")
        return

    wb = Workbook()
    summary = wb.active
    summary.title = "Summary"
    info = [
        ["Exported", datetime.now().strftime("%Y-%m-%d %H:%M:%S")],
        ["Units", "positions in mm, angles in degrees, currents in A"],
        ["Cycles in report", len(reports)],
        ["Data log samples", len(log_rows)],
        ["Supply voltage (V)", SUPPLY_VOLTAGE],
        ["Torque constant Kt (Nm/A)", KT],
        ["Current limit per servo (A)", CURRENT_LIMIT],
        ["Cube mass (kg)", CUBE_MASS],
        ["Measured data source", "ESP" if measured_is_live() else "fictitious"],
        [],
        ["Sequence steps"],
    ]
    info += [[f"{i + 1}. {describe(s).replace('**', '')}"]
             for i, s in enumerate(seq["steps"])]
    if trials_summary:
        info += [[], ["Trials summary"]]
        info += [[line.replace("**", "").strip()] for line in trials_summary.split("\n")]
    write_sheet(summary, ["Item", "Value"], info, widths=40)

    write_sheet(wb.create_sheet("Cycle reports"), REPORT_COLUMNS,
                [report_row(r) for r in reports])
    write_sheet(wb.create_sheet("Data log"), CSV_HEADER, log_rows)

    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"arm_export_{datetime.now():%Y%m%d_%H%M%S}.xlsx"
    wb.save(path)
    notify("Export saved", str(path), color="green")
    send_to_browser(event, path)


@report_clear.on_click
def _(_):
    global trials_summary
    trials_summary = ""
    reports.clear()
    report_md.content = "*No cycles run yet*"
    report_hist_md.content = render_report_history()


# =============================================================================
# WORKSPACE MAP
# =============================================================================
ws = {"running": False}
ws_data = model.createData()


def reach_test(p, q, pointing_down, iters=60, d=None):
    """Damped least-squares IK. Returns (reached, final q)."""
    d = ws_data if d is None else d
    for _ in range(iters):
        pin.framesForwardKinematics(model, d, q)
        M = d.oMf[TCP_ID]
        e_pos = p - M.translation
        if pointing_down:
            e_rot = pin.log3(R_DOWN @ M.rotation.T)
            if np.linalg.norm(e_pos) < 0.003 and np.linalg.norm(e_rot) < 0.05:
                return True, q
            err = np.concatenate([e_pos, e_rot])
        else:
            if np.linalg.norm(e_pos) < 0.003:
                return True, q
            err = e_pos
        J = pin.computeFrameJacobian(model, d, q, TCP_ID,
                                     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J = J[:len(err), :NUM_JOINTS]
        step = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(len(err)), err)
        dq = np.zeros(model.nv)
        dq[:NUM_JOINTS] = np.clip(step, -0.5, 0.5)
        q = np.clip(pin.integrate(model, q, dq),
                    model.lowerPositionLimit, model.upperPositionLimit)
    return False, q


check_data = model.createData()


def check_point(p):
    """Returns (ok, message). ok is False if the arm cannot reach p at all."""
    if p[2] < 0.0:
        return False, "❌ Below the ground (Z must be 0 mm or more)."
    if np.linalg.norm(p - SHOULDER) > MAX_REACH + 0.05:
        return False, "❌ Outside the arm's working area (too far away)."
    seeds = [configuration.q.copy(), q0.copy()]
    for seed in seeds:
        if reach_test(p, seed, True, iters=80, d=check_data)[0]:
            return True, ""
    for seed in seeds:
        if reach_test(p, seed, False, iters=80, d=check_data)[0]:
            return True, ("⚠️ Reachable, but the gripper cannot point "
                          "straight down there.")
    return False, "❌ Outside the arm's working area."


def scan_workspace(height):
    global workspace_cloud
    xs = np.arange(-WORKSPACE_RADIUS, WORKSPACE_RADIUS + 1e-9, WORKSPACE_STEP)
    points, colours = [], []
    n_down = n_pos = n_total = 0
    for row, x in enumerate(xs):
        seed = q0.copy()
        for y in xs:
            r = np.hypot(x, y)
            if r > WORKSPACE_RADIUS or r < 0.08:
                continue
            n_total += 1
            p = np.array([x, y, height])
            ok, q = reach_test(p, seed, True)
            if not ok:
                ok, q2 = reach_test(p, q0.copy(), True)
                if ok:
                    q = q2
            if ok:
                n_down += 1
                seed = q
                points.append(p)
                colours.append((60, 190, 90))
                continue
            ok, q = reach_test(p, seed, False)
            if ok:
                n_pos += 1
                seed = q
                points.append(p)
                colours.append((240, 190, 40))
        ws_progress.value = 100.0 * (row + 1) / len(xs)

    cell = WORKSPACE_STEP ** 2
    if points:
        workspace_cloud = server.scene.add_point_cloud(
            "/workspace_map", points=np.array(points, dtype=np.float32),
            colors=np.array(colours, dtype=np.uint8),
            point_size=WORKSPACE_STEP * 0.9, point_shape="square",
            point_shading="flat",
        )
    ws_md.content = (
        f"Map at Z = {height * MM:.0f} mm  \n"
        f"🟩 pointing down: **{n_down * cell:.2f} m²**  \n"
        f"🟨 position only: **{n_pos * cell:.2f} m²**  \n"
        f"Unreachable: {(n_total - n_down - n_pos) * cell:.2f} m² of the scanned area  \n"
        "*Self-collisions are not checked.*"
    )
    ws["running"] = False
    notify("Workspace scan finished", f"Height {height * MM:.0f} mm", color="green")


@ws_scan.on_click
def _(_):
    if ws["running"]:
        return
    ws["running"] = True
    ws_progress.value = 0.0
    ws_md.content = "Scanning..."
    threading.Thread(target=scan_workspace, args=(ws_height.value / MM,),
                     daemon=True).start()


@ws_clear.on_click
def _(_):
    global workspace_cloud
    if workspace_cloud is not None:
        workspace_cloud.remove()
        workspace_cloud = None
    ws_progress.value = 0.0


# =============================================================================
# SEQUENCE FILES (positions stored in mm)
# =============================================================================
def step_to_json(s):
    out = {"kind": s["kind"]}
    if s["kind"] == "move":
        out.update(name=s["name"], pos_mm=[round(float(v) * MM, 1) for v in s["pos"]])
    if s["kind"] == "wait":
        out["secs"] = s["secs"]
    return out


def step_from_json(d):
    if d["kind"] == "move":
        if "pos_mm" in d:
            pos = np.array(d["pos_mm"], dtype=float) / MM
        else:
            pos = np.array(d["pos"], dtype=float)    # older files, in metres
        return {"kind": "move", "name": d["name"], "pos": pos}
    if d["kind"] == "wait":
        return {"kind": "wait", "secs": float(d.get("secs", 1.0))}
    return {"kind": d["kind"]}


# =============================================================================
# SETUP TAB: DISPLAY
# =============================================================================
@dark_cb.on_update
def _(_):
    server.gui.configure_theme(
        dark_mode=dark_cb.value, brand_color=BRAND,
        control_width="large", show_share_button=False,
    )
    apply_scene_theme(dark_cb.value)


@show_presets.on_update
def _(_):
    for ball, label in preset_nodes.values():
        ball.visible = show_presets.value
        label.visible = show_presets.value


@show_labels.on_update
def _(_):
    for _ball, label in seq_nodes:
        label.visible = show_labels.value


@show_grid.on_update
def _(_):
    ground.visible = show_grid.value


@show_trail.on_update
def _(_):
    if trail is not None:
        trail.visible = show_trail.value


@cam_btn.on_click
def _(event):
    if event.client is not None:
        reset_camera(event.client)


# =============================================================================
# MAIN LOOP
# =============================================================================
def bar(fraction, width=10):
    n = int(round(np.clip(fraction, 0.0, 1.0) * width))
    return "▰" * n + "▱" * (width - n)


def update_reachability(tip, tp, qd_actual):
    if np.linalg.norm(tp - state["last_target"]) > 1e-6:
        state["last_target"] = tp.copy()
        state["stall_time"] = 0.0
        state["warned_target"] = None

    err_mm = np.linalg.norm(tip - tp) * 1000
    too_far = np.linalg.norm(tp - SHOULDER) > MAX_REACH
    stopped = np.max(np.abs(qd_actual[:NUM_JOINTS])) < 0.02
    state["stall_time"] = state["stall_time"] + DT if stopped else 0.0

    if err_mm < IN_POSITION_MM:
        state["reach"] = "ok"
    elif too_far or state["stall_time"] > 0.6:
        state["reach"] = "out"
    else:
        state["reach"] = "moving"

    if state["estop"]:
        colour, text = (230, 60, 60), "🛑 **Emergency stop active**"
    elif state["reach"] == "ok":
        colour, text = (60, 200, 90), f"🟢 **In position** ({err_mm:.0f} mm off)"
    elif state["reach"] == "moving":
        colour, text = (240, 190, 40), f"🟡 **Moving:** {err_mm:.0f} mm to go"
    else:
        why = "beyond the arm's reach" if too_far else "the arm cannot get closer"
        colour = (230, 60, 60)
        text = f"🔴 **Unreachable**: {why}. Closest: {err_mm:.0f} mm away."
    kind_changed = state.get("status_kind") != (state["estop"], state["reach"])
    if kind_changed or loop % 5 == 0:
        state["status_kind"] = (state["estop"], state["reach"])
        target_ball.color = colour
        reach_md.content = text
    if state["reach"] == "out" and not state["estop"]:
        if state["warned_target"] is None and state["mode"] == "Manual":
            state["warned_target"] = tp.copy()
            notify("Target unreachable", f"The closest the tip can get is "
                   f"{err_mm:.0f} mm away", color="red")
    return err_mm


def speed_cap():
    """Joint speed allowed right now: the Speed % of the max joint speed."""
    return max_speed.value * speed.value / 100.0


def limit_motion(qd_request):
    """Apply the speed and acceleration limits. Returns the velocity used."""
    qd = qd_request.copy()
    prev = state["prev_qd"]
    vmax, amax = speed_cap(), max_accel.value
    wanted_v = np.max(np.abs(qd[:NUM_JOINTS]))
    wanted_a = np.max(np.abs(qd[:NUM_JOINTS] - prev[:NUM_JOINTS])) / DT
    if wanted_v > vmax:
        qd *= vmax / wanted_v
    dq = qd - prev
    step = np.max(np.abs(dq[:NUM_JOINTS]))
    if step > amax * DT:
        qd = prev + dq * (amax * DT / step)
    return qd, wanted_v, wanted_a


def render_joints(q, qd):
    rows = ["| Joint | Angle | Speed | Range used |", "|---|---|---|---|"]
    near = []
    for j in range(NUM_JOINTS):
        frac = (q[j] - LOWER[j]) / (UPPER[j] - LOWER[j])
        margin = np.degrees(min(q[j] - LOWER[j], UPPER[j] - q[j]))
        flag = " ⚠️" if margin < 5 else ""
        if margin < 5:
            near.append(f"J{j + 1}")
        rows.append(f"| J{j + 1} | {np.degrees(q[j]):+6.1f}° | "
                    f"{qd[j]:+.2f} rad/s | {bar(frac)}{flag} |")
    if near:
        rows.append(f"\n⚠️ **Near joint limit:** {', '.join(near)}")
    return "\n".join(rows)


def render_limits(wanted_v, wanted_a):
    vcap, amax = speed_cap(), max_accel.value
    lines = [f"Speed setting: **{speed.value:.0f} %** of {max_speed.value:.2f} rad/s "
             f"= **{vcap:.2f} rad/s**  ",
             f"Requested speed: {wanted_v:.2f} rad/s "
             f"{'(slowed)' if wanted_v > vcap else ''}  ",
             f"Requested accel: {wanted_a:.1f} / {amax:.1f} rad/s² "
             f"{'(slowed)' if wanted_a > amax else ''}  ",
             "\n*The arm never moves faster than these limits.*"]
    return "\n".join(lines)


loop = 0


def step():
    """One control loop: IK, limits, hardware, currents, readouts, logging."""
    global loop, trail
    now = time.time()

    if not state["estop"] and state["mode"] == "Sequence":
        update_sequence(now)

    # --- Inverse kinematics toward the target (frozen during an E-stop) ---
    tp = np.array(target.position)
    tp[2] = max(tp[2], 0.0)
    if state["estop"]:
        qd = np.zeros(model.nv)
        wanted_v = wanted_a = 0.0
    elif state["stopping"]:
        prev = state["prev_qd"]
        fastest = np.max(np.abs(prev))
        brake = max_accel.value * DT
        # slow all joints together, so the tip keeps its direction while braking
        qd = prev * max(0.0, 1.0 - brake / fastest) if fastest > 0 else prev * 0.0
        wanted_v = wanted_a = 0.0
        configuration.integrate_inplace(qd, DT)
        target.position = tuple(tip_position())
        tp = np.array(target.position)
        if np.max(np.abs(prev)) > 0 and not np.any(qd):
            set_fields(tp)   # stopped: hold this pose until the next command
    else:
        task.set_target(pin.SE3(R_DOWN, tp))
        velocity = solve_ik(configuration, [task], DT, solver="quadprog",
                            damping=1e-3)
        qd, wanted_v, wanted_a = limit_motion(velocity)
        configuration.integrate_inplace(qd, DT)
    state["prev_qd"] = qd
    q = configuration.q
    show_robot(q)
    tip = tip_position()

    err_mm = update_reachability(tip, tp, qd)
    target_ball.position = tuple(tp)

    if state["holding"]:
        cube.position = (float(tip[0]), float(tip[1]), float(tip[2]))

    # --- Hardware: send joint angles while mirroring ---
    if mirror_cb.value and hw["mode"] == "serial" and not state["estop"]:
        angles = ",".join(f"{np.degrees(v):.1f}" for v in q[:NUM_JOINTS])
        hw_send(f"J,{angles}")

    # --- Currents ---
    tau, sim = simulated_currents(q, qd)
    fake = fictitious_measured(sim)
    live = measured_is_live()
    meas = hw["measured"].copy() if live else fake
    compare_window.append((sim.copy(), meas.copy()))
    if loop % 3 == 0:
        history.append((now, sim.copy(), meas.copy()))
    if cycle["active"]:
        cycle["samples"].append((now, sim.copy(), meas.copy(),
                                 qd[:NUM_JOINTS].copy(), float(state["holding"])))

    if loop % LOG_EVERY == 0:
        trail_points.append(tip.copy())
        if recording.value:
            log_rows.append(
                [round(now - log_start, 3), state["mode"], int(state["estop"]),
                 int(state["holding"])]
                + [round(v * MM, 1) for v in tp] + [round(v * MM, 1) for v in tip]
                + [round(np.degrees(v), 2) for v in q[:NUM_JOINTS]]
                + [round(v, 4) for v in tau]
                + [round(v, 4) for v in sim]
                + [round(v, 4) for v in meas]
                + [round(float(sim.sum()), 4), round(float(meas.sum()), 4),
                   "ESP" if live else "fictitious"]
            )

    # --- Readouts (not every loop, to keep the browser smooth) ---
    if loop % 5 == 0:
        tip_txt.value = f"{tip[0] * MM:.0f}, {tip[1] * MM:.0f}, {tip[2] * MM:.0f}"
        joints_md.content = render_joints(q, qd)
        limits_md.content = render_limits(wanted_v, wanted_a)
        currents_md.content = render_currents(sim, meas, tau)
        source_md.content = render_source()
        log_txt.value = str(len(log_rows))
        hw_status_md.content = render_hw_status()
        traffic_md.content = render_traffic()
    if loop % 10 == 0 and len(trail_points) > 1:
        pts = np.array(trail_points)
        segs = np.stack([pts[:-1], pts[1:]], axis=1)
        trail = server.scene.add_line_segments(
            "/trail", points=segs, colors=(70, 130, 230),
            thickness=2.0, thickness_units="screen", visible=show_trail.value,
        )
    if loop % 15 == 0:
        plot.figure = make_figure(dark_cb.value)

    loop += 1


def main():
    show_robot(configuration.q)
    print("Open http://localhost:8080 in your browser")
    print("Press Ctrl+C here to stop")
    try:
        while True:
            start = time.time()
            step()
            time.sleep(max(0.0, DT - (time.time() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        hw_send("S")          # leave a connected arm stopped
        hw_disconnect()


if __name__ == "__main__":
    main()
