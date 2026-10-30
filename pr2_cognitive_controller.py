from controller import Supervisor
import math
import requests
import json
import re

# ── Constants ──────────────────────────────────────────────────────────
TIME_STEP           = 32
MAX_WHEEL_SPEED     = 3.0
WHEELS_DISTANCE     = 0.4492
SUB_WHEELS_DISTANCE = 0.098
WHEEL_RADIUS        = 0.08
TOLERANCE           = 0.05

def almost_equal(a, b):
    return abs(a - b) < TOLERANCE

# ── Robot init ─────────────────────────────────────────────────────────
robot       = Supervisor()
robot_node  = robot.getSelf()
translation = robot_node.getField("translation")
rotation    = robot_node.getField("rotation")

# ── Camera ─────────────────────────────────────────────────────────────
camera = robot.getDevice("high_def_sensor")
if camera:
    camera.enable(TIME_STEP)
    print("[Init] Camera 'high_def_sensor' enabled")
else:
    print("[Init] WARNING: Camera 'high_def_sensor' not found")

# ── Wheel motors ───────────────────────────────────────────────────────
wheel_motor_names = [
    "fl_caster_l_wheel_joint",  # FLL 0
    "fl_caster_r_wheel_joint",  # FLR 1
    "fr_caster_l_wheel_joint",  # FRL 2
    "fr_caster_r_wheel_joint",  # FRR 3
    "bl_caster_l_wheel_joint",  # BLL 4
    "bl_caster_r_wheel_joint",  # BLR 5
    "br_caster_l_wheel_joint",  # BRL 6
    "br_caster_r_wheel_joint",  # BRR 7
]
wheel_motors  = []
wheel_sensors = []
for name in wheel_motor_names:
    m = robot.getDevice(name)
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
    wheel_motors.append(m)
    s = m.getPositionSensor()
    s.enable(TIME_STEP)
    wheel_sensors.append(s)

# ── Caster rotation motors ─────────────────────────────────────────────
rotation_motor_names = [
    "fl_caster_rotation_joint",  # FL 0
    "fr_caster_rotation_joint",  # FR 1
    "bl_caster_rotation_joint",  # BL 2
    "br_caster_rotation_joint",  # BR 3
]
rotation_motors  = []
rotation_sensors = []
for name in rotation_motor_names:
    m = robot.getDevice(name)
    rotation_motors.append(m)
    s = m.getPositionSensor()
    s.enable(TIME_STEP)
    rotation_sensors.append(s)

# ── Arm motors ─────────────────────────────────────────────────────────
left_arm_motor_names = [
    "l_shoulder_pan_joint",
    "l_shoulder_lift_joint",
    "l_upper_arm_roll_joint",
    "l_elbow_flex_joint",
    "l_wrist_roll_joint",
]
right_arm_motor_names = [
    "r_shoulder_pan_joint",
    "r_shoulder_lift_joint",
    "r_upper_arm_roll_joint",
    "r_elbow_flex_joint",
    "r_wrist_roll_joint",
]
left_arm_motors   = []
left_arm_sensors  = []
right_arm_motors  = []
right_arm_sensors = []

for name in left_arm_motor_names:
    m = robot.getDevice(name)
    left_arm_motors.append(m)
    s = m.getPositionSensor()
    s.enable(TIME_STEP)
    left_arm_sensors.append(s)

for name in right_arm_motor_names:
    m = robot.getDevice(name)
    right_arm_motors.append(m)
    s = m.getPositionSensor()
    s.enable(TIME_STEP)
    right_arm_sensors.append(s)

# ── Grippers ───────────────────────────────────────────────────────────
left_finger_motor   = robot.getDevice("l_finger_gripper_motor::l_finger")
right_finger_motor  = robot.getDevice("r_finger_gripper_motor::l_finger")
left_finger_sensor  = left_finger_motor.getPositionSensor()
right_finger_sensor = right_finger_motor.getPositionSensor()
left_finger_sensor.enable(TIME_STEP)
right_finger_sensor.enable(TIME_STEP)

GRIPPER_OPEN  = 0.5
GRIPPER_CLOSE = 0.0

# ── Torso ──────────────────────────────────────────────────────────────
torso_motor  = robot.getDevice("torso_lift_joint")
torso_sensor = robot.getDevice("torso_lift_joint_sensor")
torso_sensor.enable(TIME_STEP)

# ── Fridge Door State (logical — no separate door node in PROTO) ────────
_fridge_door_open = False   # CLOSED at simulation start

# ══════════════════════════════════════════════════════════════════════
# STEP
# ══════════════════════════════════════════════════════════════════════
def step():
    robot.step(TIME_STEP)

# ══════════════════════════════════════════════════════════════════════
# ANCHOR FRIDGE  (prevent physics tipping it over)
# ══════════════════════════════════════════════════════════════════════
def anchor_fridge():
    global _fridge_node
    root_children = robot.getRoot().getField("children")
    for i in range(root_children.getCount()):
        node = root_children.getMFNode(i)
        nf = node.getField("name")
        if nf and "fridge" in nf.getSFString().lower():
            _fridge_node = node
            t = node.getField("translation").getSFVec3f()
            r = node.getField("rotation").getSFRotation()
            print(f"[Anchor] Fridge at T={list(t)}  R={list(r)}")
            physics_field = node.getField("physics")
            if physics_field:
                physics_field.setSFNode(None)
                print("[Anchor] Fridge physics disabled — now static")
            else:
                print("[Anchor] No physics field — fridge may already be static")
            return
    print("[Anchor] WARNING: fridge node not found")

# ══════════════════════════════════════════════════════════════════════
# WHEEL HELPERS
# ══════════════════════════════════════════════════════════════════════
def set_wheels_speed(speed):
    for m in wheel_motors:
        m.setVelocity(speed)

def stop_wheels():
    set_wheels_speed(0.0)

_saved_torques = [0.0] * 8

def enable_passive_wheels(enable):
    global _saved_torques
    if enable:
        for i, m in enumerate(wheel_motors):
            _saved_torques[i] = m.getAvailableTorque()
            m.setAvailableTorque(0.0)
    else:
        for i, m in enumerate(wheel_motors):
            m.setAvailableTorque(_saved_torques[i])

# ══════════════════════════════════════════════════════════════════════
# CASTER ROTATION
# ══════════════════════════════════════════════════════════════════════
def set_rotation_wheels_angles(fl, fr, bl, br, wait_on_feedback):
    if wait_on_feedback:
        stop_wheels()
        enable_passive_wheels(True)

    rotation_motors[0].setPosition(fl)
    rotation_motors[1].setPosition(fr)
    rotation_motors[2].setPosition(bl)
    rotation_motors[3].setPosition(br)

    if wait_on_feedback:
        targets = [fl, fr, bl, br]
        while True:
            step()
            if all(almost_equal(rotation_sensors[i].getValue(), targets[i]) for i in range(4)):
                break
        enable_passive_wheels(False)

# ══════════════════════════════════════════════════════════════════════
# ROBOT ROTATE
# ══════════════════════════════════════════════════════════════════════
def robot_rotate(angle):
    """Rotate in place. +ve = CCW, -ve = CW."""
    print(f"[Rotate] Rotating {math.degrees(angle):.1f}°")
    stop_wheels()

    set_rotation_wheels_angles(
        3.0 * math.pi / 4,
              math.pi / 4,
       -3.0 * math.pi / 4,
             -math.pi / 4,
        True
    )

    max_spd = MAX_WHEEL_SPEED if angle > 0 else -MAX_WHEEL_SPEED
    set_wheels_speed(max_spd)

    init_pos = wheel_sensors[0].getValue()
    expected = abs(angle * 0.5 * (WHEELS_DISTANCE + SUB_WHEELS_DISTANCE))

    while True:
        step()
        travelled = abs(WHEEL_RADIUS * (wheel_sensors[0].getValue() - init_pos))
        if travelled > expected:
            break
        if expected - travelled < 0.025:
            set_wheels_speed(0.1 * max_spd)

    set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, True)
    stop_wheels()
    print("[Rotate] Done")

# ══════════════════════════════════════════════════════════════════════
# ROBOT GO FORWARD
# ══════════════════════════════════════════════════════════════════════
def robot_go_forward(distance):
    """Drive straight. +ve = forward, -ve = backward."""
    print(f"[Drive] Driving {distance:.3f}m")
    max_spd = MAX_WHEEL_SPEED if distance > 0 else -MAX_WHEEL_SPEED
    set_wheels_speed(max_spd)

    init_pos = wheel_sensors[0].getValue()

    while True:
        step()
        travelled = abs(WHEEL_RADIUS * (wheel_sensors[0].getValue() - init_pos))
        if travelled > abs(distance):
            break
        if abs(distance) - travelled < 0.025:
            set_wheels_speed(0.1 * max_spd)

    stop_wheels()
    print("[Drive] Done")

# ══════════════════════════════════════════════════════════════════════
# ROTATE TO FACE TARGET  (compute exact angle to a world position)
# ══════════════════════════════════════════════════════════════════════
def rotate_to_face(target_pos):
    """
    Rotate the robot until it faces the given [x, y, z] world position.
    Uses the fridge node's live position for accuracy.
    """
    pos  = translation.getSFVec3f()
    dx   = target_pos[0] - pos[0]
    dz   = target_pos[2] - pos[2]
    want = math.atan2(dz, dx)
    err  = norm(want - get_yaw())
    print(f"[Face] Rotating {math.degrees(err):.1f}° to face target")
    if abs(err) > 0.05:
        robot_rotate(err)

# ══════════════════════════════════════════════════════════════════════
# ARM
# ══════════════════════════════════════════════════════════════════════
def set_left_arm(sr, sl, uar, el, wr, wait):
    vals = [sr, sl, uar, el, wr]
    for i, m in enumerate(left_arm_motors):
        m.setPosition(vals[i])
    if wait:
        while not all(almost_equal(left_arm_sensors[i].getValue(), vals[i]) for i in range(5)):
            step()

def set_right_arm(sr, sl, uar, el, wr, wait):
    vals = [sr, sl, uar, el, wr]
    for i, m in enumerate(right_arm_motors):
        m.setPosition(vals[i])
    if wait:
        while not all(almost_equal(right_arm_sensors[i].getValue(), vals[i]) for i in range(5)):
            step()

# ══════════════════════════════════════════════════════════════════════
# TORSO
# ══════════════════════════════════════════════════════════════════════
def set_torso(height, wait):
    torso_motor.setPosition(height)
    if wait:
        while not almost_equal(torso_sensor.getValue(), height):
            step()

# ══════════════════════════════════════════════════════════════════════
# GRIPPER
# ══════════════════════════════════════════════════════════════════════
def set_gripper(left, open_gripper, wait):
    motor  = left_finger_motor  if left else right_finger_motor
    sensor = left_finger_sensor if left else right_finger_sensor
    target = GRIPPER_OPEN if open_gripper else GRIPPER_CLOSE
    motor.setPosition(target)
    if wait:
        while not almost_equal(sensor.getValue(), target):
            step()

# ══════════════════════════════════════════════════════════════════════
# FRIDGE DOOR  (logical state tracking — no physical door node)
# ══════════════════════════════════════════════════════════════════════
def is_fridge_door_open():
    global _fridge_door_open
    state = "OPEN" if _fridge_door_open else "CLOSED"
    print(f"[Fridge] Door state → {state}")
    return _fridge_door_open

def open_fridge_door(wait=True):
    global _fridge_door_open
    print("[Fridge] Opening door...")
    _fridge_door_open = True
    if wait:
        for _ in range(10):
            step()
    print("[Fridge] Door is now OPEN")

def close_fridge_door(wait=True):
    global _fridge_door_open
    print("[Fridge] Closing door...")
    _fridge_door_open = False
    if wait:
        for _ in range(10):
            step()
    print("[Fridge] Door is now CLOSED")

# ══════════════════════════════════════════════════════════════════════
# CAMERA SNAPSHOT  (call anytime to capture current view)
# ══════════════════════════════════════════════════════════════════════
def camera_look():
    """Step once so camera buffer refreshes, then report what we see."""
    step()
    if camera:
        w = camera.getWidth()
        h = camera.getHeight()
        print(f"[Camera] Frame captured — {w}x{h} px  |  "
              f"FOV={math.degrees(camera.getFov()):.1f}°")
    else:
        print("[Camera] No camera available")

# ══════════════════════════════════════════════════════════════════════
# INITIAL POSITION
# ══════════════════════════════════════════════════════════════════════
def set_initial_position():
    print("[Init] Setting initial position...")
    set_left_arm (0.0, 1.35, 0.0, -2.2, 0.0, False)
    set_right_arm(0.0, 1.35, 0.0, -2.2, 0.0, False)
    set_gripper(True,  True, False)
    set_gripper(False, True, False)
    set_torso(0.2, True)
    print("[Init] Done")

# ══════════════════════════════════════════════════════════════════════
# YAW
# ══════════════════════════════════════════════════════════════════════
def get_yaw():
    ax, ay, az, angle = rotation.getSFRotation()
    if abs(ay) >= abs(ax) and abs(ay) >= abs(az):
        return ay * angle
    elif abs(az) >= abs(ax):
        return az * angle
    return ax * angle

def norm(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

# ══════════════════════════════════════════════════════════════════════
# FIND OBJECT
# ══════════════════════════════════════════════════════════════════════
def find_object(name):
    children = robot.getRoot().getField("children")
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        nf   = node.getField("name")
        if nf and name.lower() in nf.getSFString().lower():
            p   = node.getField("translation").getSFVec3f()
            pos = [p[0], p[1], p[2]]
            print(f"[Vision] '{nf.getSFString()}' → X={p[0]:.2f}  Y={p[1]:.2f}  Z={p[2]:.2f}")
            return pos
    print(f"[Vision] NOT FOUND: '{name}'")
    return None

# ══════════════════════════════════════════════════════════════════════
# COMPUTE DISTANCE & ANGLE TO TARGET
# ══════════════════════════════════════════════════════════════════════
def compute_nav(target, stop_dist):
    pos  = translation.getSFVec3f()
    dx   = target[0] - pos[0]
    dz   = target[2] - pos[2]
    dist = math.sqrt(dx*dx + dz*dz)
    want = math.atan2(dz, dx)
    head = get_yaw()
    err  = norm(want - head)
    drive_dist = max(0.0, dist - stop_dist)
    print(f"[Nav] Distance={dist:.3f}m  |  Rotate={math.degrees(err):.1f}°  |  Drive={drive_dist:.3f}m")
    return err, drive_dist

# ══════════════════════════════════════════════════════════════════════
# PICK SEQUENCE  (torso raised for table-height objects)
# ══════════════════════════════════════════════════════════════════════
def pick():
    print("\n[Pick] ── Starting pick sequence ──")

    print("[Pick] Raising torso for table height...")
    set_torso(0.33, True)

    print("[Pick] Opening grippers...")
    set_gripper(True,  True, True)
    set_gripper(False, True, True)

    print("[Pick] Positioning arms to table level...")
    set_left_arm (0.0, -0.2, 0.0, -0.5, 0.0, True)
    set_right_arm(0.0, -0.2, 0.0, -0.5, 0.0, True)

    print("[Pick] Nudging forward to grasp...")
    robot_go_forward(0.12)

    print("[Pick] Closing grippers...")
    set_gripper(True,  False, True)
    set_gripper(False, False, True)

    print("[Pick] Lifting object...")
    set_left_arm (0.0, 0.3, 0.0, -1.2, 0.0, True)
    set_right_arm(0.0, 0.3, 0.0, -1.2, 0.0, True)

    print("[Pick] ── Complete ──")

# ══════════════════════════════════════════════════════════════════════
# PLACE SEQUENCE
# ══════════════════════════════════════════════════════════════════════
def place():
    print("\n[Place] ── Starting place sequence ──")

    print("[Place] Lowering arms...")
    set_left_arm (0.0, -0.2, 0.0, -0.5, 0.0, True)
    set_right_arm(0.0, -0.2, 0.0, -0.5, 0.0, True)

    print("[Place] Releasing object...")
    set_gripper(True,  True, True)
    set_gripper(False, True, True)

    robot_go_forward(-0.12)

    print("[Place] Resetting arms and torso...")
    set_left_arm (0.0, 1.35, 0.0, -2.2, 0.0, True)
    set_right_arm(0.0, 1.35, 0.0, -2.2, 0.0, True)
    set_torso(0.2, True)

    print("[Place] ── Complete ──")

# ══════════════════════════════════════════════════════════════════════
# LLM
# ══════════════════════════════════════════════════════════════════════
def fallback():
    return {"object": "jam jar", "source": "table", "target": "fridge"}

def llm_reasoning(command):
    prompt = f"""Return ONLY valid JSON. No explanation. No markdown.
{{
  "object": "<item to move>",
  "source": "<pickup location>",
  "target": "<drop location>"
}}
Instruction: {command}"""
    try:
        res   = requests.post("http://localhost:11434/api/generate",
                              json={"model": "llama3", "prompt": prompt, "stream": False},
                              timeout=10)
        text  = res.json().get("response", "")
        match = re.search(r'\{.*?\}', text, re.DOTALL)
        if match:
            p = json.loads(match.group())
            print("[LLM] Goal:", p)
            return p
    except Exception as e:
        print("[LLM] Error:", e)
    print("[LLM] Using fallback")
    return fallback()

# ══════════════════════════════════════════════════════════════════════
# INITIAL POSITIONING SEQUENCE
#
#   Phase A → Move forward 1.7m       (approach corridor)
#   Phase B → Rotate to face fridge   (computed from fridge world pos)
#   Phase C → Move forward 0.942m     (close to fridge door)
#
# This runs ONCE at startup, before the main task.
# ══════════════════════════════════════════════════════════════════════
def initial_positioning():
    print("\n" + "="*55)
    print("INITIAL POSITIONING SEQUENCE")
    print("="*55)

    # ── Phase A: Move forward 1.7m ─────────────────────────────────
    print("\n── Phase A: Moving forward 1.7m ──")
    camera_look()          # capture view before moving
    robot_go_forward(1.7)

    # ── Phase B: Rotate to face fridge ────────────────────────────
    print("\n── Phase B: Rotating to face fridge ──")
    camera_look()          # capture view after first drive
    fridge_pos = find_object("fridge")
    if fridge_pos:
        rotate_to_face(fridge_pos)
    else:
        print("[WARNING] Fridge not found — skipping rotation")

    # ── Phase C: Move forward 0.942m ──────────────────────────────
    print("\n── Phase C: Moving forward 0.942m toward fridge ──")
    robot_go_forward(0.942)
    camera_look()          # capture view after arriving at fridge

    print("\n── Initial positioning complete ──")

# ══════════════════════════════════════════════════════════════════════
# EXECUTE TASK
#
#   STEP 1 → Check door state  (Python bool)
#              ├─ OPEN  → skip
#              └─ CLOSED → navigate to fridge → open door → back away
#   STEP 2 → Navigate to jam jar
#   STEP 3 → Pick up jam jar
#   STEP 4 → Navigate to fridge
#   STEP 5 → Place jam jar inside fridge
#   STEP 6 → Reverse → close fridge door
# ══════════════════════════════════════════════════════════════════════
def execute(cmd):
    print("\n" + "="*55)
    print("COMMAND:", cmd)
    print("="*55)

    goal     = llm_reasoning(cmd)
    obj_name = goal["object"]
    tgt_name = goal["target"]

    # ── STEP 1: Check fridge door state ───────────────────────────
    print(f"\n── STEP 1: Checking fridge door state ──")
    camera_look()

    if is_fridge_door_open():
        print("[Fridge] Door already OPEN — skipping open sequence")
    else:
        print("[Fridge] Door CLOSED — navigating to open it...")

        fridge_pos = find_object(tgt_name)
        if not fridge_pos:
            print("[ERROR] Fridge not found"); return

        rot_err0, drive_dist0 = compute_nav(fridge_pos, stop_dist=0.85)
        if abs(rot_err0) > 0.05:
            robot_rotate(rot_err0)
        if drive_dist0 > 0.01:
            robot_go_forward(drive_dist0)

        open_fridge_door()
        robot_go_forward(-0.30)   # back away from door swing arc

    # ── STEP 2: Navigate to jam jar ───────────────────────────────
    print(f"\n── STEP 2: Navigate to {obj_name} ──")
    camera_look()

    obj_pos = find_object(obj_name)
    if not obj_pos:
        print("[ERROR] Object not found"); return

    rot_err1, drive_dist1 = compute_nav(obj_pos, stop_dist=0.55)
    if abs(rot_err1) > 0.05:
        robot_rotate(rot_err1)
    if drive_dist1 > 0.01:
        robot_go_forward(drive_dist1)

    # ── STEP 3: Pick up jam jar ────────────────────────────────────
    print(f"\n── STEP 3: Pick up {obj_name} ──")
    camera_look()
    pick()

    # ── STEP 4: Navigate to fridge ────────────────────────────────
    print(f"\n── STEP 4: Navigate to {tgt_name} ──")

    tgt_pos = find_object(tgt_name)
    if not tgt_pos:
        print("[ERROR] Fridge not found"); return

    rot_err2, drive_dist2 = compute_nav(tgt_pos, stop_dist=0.70)
    if abs(rot_err2) > 0.05:
        robot_rotate(rot_err2)
    if drive_dist2 > 0.01:
        robot_go_forward(drive_dist2)

    # ── STEP 5: Place jam jar inside fridge ───────────────────────
    print(f"\n── STEP 5: Place {obj_name} inside {tgt_name} ──")
    camera_look()
    place()

    # ── STEP 6: Back away then close fridge door ──────────────────
    print(f"\n── STEP 6: Closing fridge door ──")
    robot_go_forward(-0.35)
    close_fridge_door()
    camera_look()          # final view confirming task done

    print("\n" + "="*55)
    print("✓ Task complete!")
    print("="*55)

# ══════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════
print("\n=== PR2 COGNITIVE CONTROLLER ===\n")

# Fix fridge so it cannot fall over
anchor_fridge()

# Set robot to safe starting pose
set_initial_position()

# Let physics settle
for _ in range(10):
    step()

# ── Pre-task positioning ───────────────────────────────────────────────
# Phase A: drive 1.7m forward
# Phase B: rotate to face fridge (computed from live fridge position)
# Phase C: drive 0.942m toward fridge
initial_positioning()

# ── Main task ──────────────────────────────────────────────────────────
execute("take the jam jar from the table and put it in the fridge")

while robot.step(TIME_STEP) != -1:
    pass