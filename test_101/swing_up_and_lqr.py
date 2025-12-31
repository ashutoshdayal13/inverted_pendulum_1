# combined_swingup_lqr_fallback.py

from nmotion_transport import USBInterface, Driver, NCoder
import time, math, csv
import numpy as np
import sys

# ==========================
# Hardware & physical params
# ==========================
COM_PORT = "COM3"
DRIVER_ID = 1
ENCODER_ID = 2

PULLEY_RADIUS = 0.0075
KT = 0.0306
MAX_CURRENT = 8.0

# Pendulum
M = 0.2
L = 0.15
G = 9.81

# ==========================
# Swing-up parameters (UNCHANGED)
# ==========================
K_ENERGY = 25.0
K_DAMP   = 0.55
K_CENTER = 0.0
K_XDOT   = 0.9

WALL_DIST = 1.20
K_WALL    = 50.0
D_WALL    = 6.0

FREQ = 200
DT = 1.0 / FREQ
ALPHA_VEL = 0.001

# ==========================
# LQR parameters (UNCHANGED)
# ==========================
UPRIGHT_OFFSET = 146.206
ALPHA = 0.18
K = np.array([-10., -11.77490462, 50.73172986, 6.3])
LQR_POS_LIMIT = 13

# Fallback thresholds
FALL_ANGLE = 45.0       # deg
FALL_COUNT_MAX = 5

# ==========================
# Helpers
# ==========================
class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.prev = 0.0
    def update(self, v):
        self.prev = self.alpha * v + (1 - self.alpha) * self.prev
        return self.prev

def clipped(v, lo, hi):
    return max(lo, min(hi, v))

def map_angle(raw):
    return (raw - 146.09 + 180) % 360 - 180

# ==========================
# Hardware init
# ==========================
iface = USBInterface(COM_PORT)
drv = Driver(DRIVER_ID, iface)
enc = NCoder(ENCODER_ID, iface)

# ==========================
# Swing-up controller
# ==========================
def run_swingup():
    print("[SWING-UP] Running")

    start_rot = None
    while start_rot is None:
        s2, r = drv.getMotorPosition()
        if s2 == 0:
            start_rot = r
        time.sleep(0.01)

    vel_filt = LowPassFilter(ALPHA_VEL)
    prev_theta = None
    prev_x = None
    E_target = M * G * L

    while True:
        s1, raw_angle = enc.getAbsoluteAngle()
        s2, raw_rot = drv.getMotorPosition()
        if s1 != 0 or s2 != 0:
            drv.setTorqueControl(0.0, 0)
            continue

        theta_deg = map_angle(raw_angle)
        theta = math.radians(theta_deg)
        x = (raw_rot - start_rot) * (2 * math.pi * PULLEY_RADIUS)

        if prev_theta is None:
            prev_theta = theta
            prev_x = x
            continue

        raw_w = (theta - prev_theta) / DT
        if raw_w > 50: raw_w -= 2 * math.pi / DT
        elif raw_w < -50: raw_w += 2 * math.pi / DT

        theta_dot = vel_filt.update(raw_w)
        x_dot = (x - prev_x) / DT
        prev_theta = theta
        prev_x = x

        PE = M * G * L * math.cos(theta)
        KE = 0.5 * M * L**2 * theta_dot**2
        E = PE + KE
        E_err = E_target - E

        u_energy = K_ENERGY * E_err * theta_dot * math.cos(theta)
        u_damp = -K_DAMP * theta_dot
        u_center = -K_CENTER * x - K_XDOT * x_dot
        u = u_energy + u_damp + u_center

        u_wall = 0.0
        if x > WALL_DIST:
            u_wall = -K_WALL * (x - WALL_DIST) - D_WALL * x_dot
            if u > 0: u = 0
        elif x < -WALL_DIST:
            u_wall = -K_WALL * (x + WALL_DIST) - D_WALL * x_dot
            if u < 0: u = 0

        current = clipped(u + (u_wall * PULLEY_RADIUS) / KT,
                          -MAX_CURRENT, MAX_CURRENT)

        drv.setTorqueControl(current, 3000)

        if abs(theta_deg) < 15 and abs(theta_dot) < 1:
            drv.setTorqueControl(0.0, 0)
            print("[SWING-UP] Success → LQR")
            return start_rot

# ==========================
# LQR controller with fallback
# ==========================
def run_lqr(start_rot):
    print("[LQR] Engaged")

    state = np.zeros(4)
    avg_x_dot = 0.0
    avg_theta_dot = 0.0
    fall_count = 0
    prev_time = time.perf_counter()

    while True:
        now = time.perf_counter()
        dt = now - prev_time
        if dt <= 1e-4:
            continue

        s1, raw_angle = enc.getAbsoluteAngle()
        s2, raw_rot = drv.getMotorPosition()
        if s1 != 0 or s2 != 0:
            continue

        theta_deg = raw_angle - UPRIGHT_OFFSET
        while theta_deg > 180: theta_deg -= 360
        while theta_deg < -180: theta_deg += 360
        theta = math.radians(theta_deg)

        x = (raw_rot - start_rot) * (2 * math.pi * PULLEY_RADIUS)
        raw_x_dot = (x - state[0]) / dt
        raw_theta_dot = (theta - state[2]) / dt

        avg_x_dot = ALPHA * raw_x_dot + (1 - ALPHA) * avg_x_dot
        avg_theta_dot = ALPHA * raw_theta_dot + (1 - ALPHA) * avg_theta_dot

        state = np.array([x, avg_x_dot, theta, avg_theta_dot])

        # --- FALL DETECTION ---
        if abs(theta_deg) > FALL_ANGLE:
            fall_count += 1
        else:
            fall_count = 0

        if fall_count >= FALL_COUNT_MAX:
            drv.setTorqueControl(0.0, 0)
            print("[LQR] Fell → Back to swing-up")
            return

        # --- LQR CONTROL ---
        force = -np.dot(K, state)
        current = clipped((force * PULLEY_RADIUS) / KT,
                          -MAX_CURRENT, MAX_CURRENT)
        drv.setTorqueControl(current, 3000)

        prev_time = now

# ==========================
# MAIN FSM LOOP
# ==========================
if __name__ == "__main__":
    try:
        while True:
            start_rot = run_swingup()
            run_lqr(start_rot)
    except KeyboardInterrupt:
        print("[ABORTED]")
    finally:
        drv.setDeviceToIdle()
        print("[STOPPED]")
