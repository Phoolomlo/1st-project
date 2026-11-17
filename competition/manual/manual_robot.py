# MakeX manual robot - CyberPi + mBot2 Shield + Bluetooth controller
# Upload mode (MicroPython). In mBlock 5: add the extensions
# "mBot2 Shield" and "Bluetooth Controller", then switch to Upload mode.
#
# Drive: 2 encoder motors (EM1 left, EM2 right) + caster
# Launcher: DC motor on DC1
# Gripper:  servo on S1
# Lift:     servo on S2

import cyberpi
import mbot2
import time

# =================== PORTS ===================
EM_LEFT = "EM1"
EM_RIGHT = "EM2"
LAUNCHER_PORT = "DC1"
GRIPPER_PORT = "S1"
LIFT_PORT = "S2"

# =================== TUNING ===================
DRIVE_MAX = 100          # top drive power, percent
TURN_MAX = 70            # top turning power - lower so turns stay controllable
SLOW_FACTOR = 0.4        # precision mode multiplier, for lining up on the cube
DEADBAND = 25            # joystick noise below this counts as centre (axis is -255..255)
AXIS_MAX = 255.0

LAUNCHER_POWER = 100     # flywheel power when firing
LAUNCHER_SPINUP = 0.6    # seconds to reach speed before a shot counts

GRIPPER_OPEN = 20        # servo angles - measure these on your own build
GRIPPER_SHUT = 110
LIFT_DOWN = 0
LIFT_UP = 90
LIFT_STEP = 3            # degrees per loop while the lift button is held

LOOP_DELAY = 0.02        # 50 Hz control loop

# =================== GAMEPAD ===================
# EVERY read of the controller happens in here, so if the API names are
# different in your mBlock version this is the only block to change.
#
# To get the exact names: build one of these reads with blocks in mBlock 5,
# then press the block-to-Python button and copy what it generates.
#
# Axes report roughly -255..+255. Buttons report True/False.

def read_gamepad():
    g = {}
    g["lx"] = cyberpi.controller.get_joystick("lx")
    g["ly"] = cyberpi.controller.get_joystick("ly")
    g["ry"] = cyberpi.controller.get_joystick("ry")
    g["fire"] = cyberpi.controller.is_press("r1")
    g["grip"] = cyberpi.controller.is_press("l1")
    g["lift_up"] = cyberpi.controller.is_press("1")
    g["lift_dn"] = cyberpi.controller.is_press("2")
    g["slow"] = cyberpi.controller.is_press("l2")
    return g

def axis(value):
    # Raw axis -> -1.0..+1.0 with the deadband removed, so a resting
    # joystick can't creep the robot.
    if value is None:
        return 0.0
    if abs(value) < DEADBAND:
        return 0.0
    span = AXIS_MAX - DEADBAND
    out = (abs(value) - DEADBAND) / span
    if out > 1.0:
        out = 1.0
    if value < 0:
        out = -out
    return out

# =================== DRIVE ===================
def drive(forward, turn, slow):
    # Arcade mixing: one stick axis for forward, one for turn.
    scale = SLOW_FACTOR if slow else 1.0
    left = (forward * DRIVE_MAX + turn * TURN_MAX) * scale
    right = (forward * DRIVE_MAX - turn * TURN_MAX) * scale
    left = clamp(left, -100, 100)
    right = clamp(right, -100, 100)
    mbot2.drive_power(left, right)

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def stop_all():
    mbot2.drive_power(0, 0)
    mbot2.motor_set(0, LAUNCHER_PORT)

# =================== MECHANISMS ===================
class Launcher:
    # Hold the fire button to spin up and shoot; release to stop.
    def __init__(self):
        self.running = False
        self.started_at = 0

    def update(self, want_fire):
        if want_fire and not self.running:
            mbot2.motor_set(LAUNCHER_POWER, LAUNCHER_PORT)
            self.running = True
            self.started_at = time.time()
        elif not want_fire and self.running:
            mbot2.motor_set(0, LAUNCHER_PORT)
            self.running = False

    def ready(self):
        # True once the flywheel has had time to reach full speed.
        return self.running and (time.time() - self.started_at) > LAUNCHER_SPINUP

class Gripper:
    # Held closed while the button is down, so a knock can't drop the cube.
    def __init__(self):
        self.shut = False
        mbot2.servo_set(GRIPPER_OPEN, GRIPPER_PORT)

    def update(self, want_shut):
        if want_shut != self.shut:
            mbot2.servo_set(GRIPPER_SHUT if want_shut else GRIPPER_OPEN, GRIPPER_PORT)
            self.shut = want_shut

class Lift:
    # Moves a few degrees per loop while held, so you can stop at any height.
    def __init__(self):
        self.angle = LIFT_DOWN
        mbot2.servo_set(self.angle, LIFT_PORT)

    def update(self, up, down):
        target = self.angle
        if up:
            target = self.angle + LIFT_STEP
        elif down:
            target = self.angle - LIFT_STEP
        target = clamp(target, LIFT_DOWN, LIFT_UP)
        if target != self.angle:
            self.angle = target
            mbot2.servo_set(self.angle, LIFT_PORT)

# =================== MAIN ===================
def main():
    cyberpi.console.clear()
    cyberpi.display.set_brush(0, 255, 0)
    cyberpi.console.println("manual robot ready")
    stop_all()

    launcher = Launcher()
    gripper = Gripper()
    lift = Lift()
    lost_frames = 0

    while True:
        try:
            g = read_gamepad()
            lost_frames = 0
        except Exception:
            # Controller dropped out - stop moving rather than run away.
            lost_frames = lost_frames + 1
            if lost_frames > 3:
                stop_all()
            time.sleep(LOOP_DELAY)
            continue

        forward = axis(g["ly"])
        turn = axis(g["lx"])
        drive(forward, turn, g["slow"])

        launcher.update(g["fire"])
        gripper.update(g["grip"])
        lift.update(g["lift_up"], g["lift_dn"])

        time.sleep(LOOP_DELAY)

main()
