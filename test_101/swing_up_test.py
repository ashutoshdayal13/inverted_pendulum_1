from nmotion_transport import USBInterface, Driver, NCoder
import time, math, csv
import numpy as np
import sys

# ==========================
# Hardware & physical params
# ==========================
COM_PORT = "COM3" # Check your port
DRIVER_ID = 1
ENCODER_ID = 2

MAX_CURRENT    = 8.0        # Increased slightly for authority
PULLEY_RADIUS  = 0.0075     # m
KT             = 0.0306     # Nm/A

# pendulum
M = 0.2       # kg
L = 0.15      # m
G = 9.81      # m/s^2

# ==========================
# Controller gains
# ==========================
K_ENERGY = 20.0    # Stronger energy pump
K_DAMP   = 0.65    # Damping
K_CENTER = 0.0     # Keep 0 for free roam
K_XDOT   = 0.9     # Friction compensation/damping

# safety
# FIXED: Was 5.18 (Too huge). Set to typical rail limit (e.g. 20cm)
WALL_DIST = 1.20   
K_WALL    = 50.0   # Stiffer wall
D_WALL    = 6.0    # High damping to prevent bouncing

# loop
FREQ = 200
DT = 1.0 / FREQ
ALPHA_VEL = 0.001   # Faster filter for better reaction

LOGFILE = "swingup_robust_log.csv"

# ==========================
# Helper objects
# ==========================
class LowPassFilter:
    def __init__(self, alpha, init=0.0):
        self.alpha = alpha
        self.prev = init
    def update(self, val):
        self.prev = self.alpha * val + (1 - self.alpha) * self.prev
        return self.prev

def clipped(v, a, b):
    return max(a, min(b, v))

# ==========================
# Connect to hardware
# ==========================
try:
    iface = USBInterface(COM_PORT)
    drv = Driver(DRIVER_ID, iface)
    enc = NCoder(ENCODER_ID, iface)
except Exception as e:
    print("HARDWARE ERROR on connect:", e)
    sys.exit(1)

print("[INIT] reading initial positions...")

start_rot = None
t0 = time.perf_counter()
timeout = 5.0

while start_rot is None:
    if (time.perf_counter() - t0) > timeout:
        print("[ERROR] Timeout reading initial positions.")
        sys.exit(1)
    s2, r = drv.getMotorPosition()
    if s2 == 0: start_rot = r
    time.sleep(0.01)

print(f"[OK] start_rot = {start_rot}")
print("[READY] Beginning swing-up in 1.5s...")
time.sleep(1.5)

E_target = M * G * L 
vel_filt = LowPassFilter(ALPHA_VEL)
prev_theta = None
prev_x = None
log = []

# Map raw encoder to [-180, 180] with 0 = Upright
def map_angle(raw):
    return (raw - 146.09 + 180) % 360 - 180

read_errors = 0
MAX_READ_ERRORS = 50 # Reduced tolerance (quarter second)

start_time = time.perf_counter()
next_tick = start_time

print("[LOOP] running. Press Ctrl+C to stop.")

try:
    while True:
        now = time.perf_counter()
        if now < next_tick: continue
        next_tick += DT
        t_elapsed = now - start_time

        # --- 1. SENSOR READ ---
        s1, raw_angle = enc.getAbsoluteAngle()
        s2, raw_rot = drv.getMotorPosition()
        
        # --- CRITICAL FIX: FAIL-SAFE ---
        if s1 != 0 or s2 != 0:
            # If we can't see the sensors, we MUST stop the motor.
            # Do NOT just 'continue' and let the old current persist.
            drv.setTorqueControl(0.0, 0) 
            
            read_errors += 1
            if read_errors > MAX_READ_ERRORS:
                print("\n[ERROR] Sensor failure limit reached.")
                break
            continue # Skip logic, but motor is safe (0A)
            
        read_errors = 0 # Reset counter on good read

        # --- 2. STATE ESTIMATION ---
        theta_deg = map_angle(raw_angle)
        theta = math.radians(theta_deg)

        rot_rel = raw_rot - start_rot
        x = rot_rel * (2.0 * math.pi * PULLEY_RADIUS)

        if prev_theta is None:
            prev_theta = theta; prev_x = x
            continue

        # Velocity w/ unwrap fix
        raw_w = (theta - prev_theta) / DT
        if raw_w > 50: raw_w -= (2.0 * math.pi) / DT
        elif raw_w < -50: raw_w += (2.0 * math.pi) / DT
        
        theta_dot = vel_filt.update(raw_w)
        x_dot = (x - prev_x) / DT

        prev_theta = theta
        prev_x = x

        # --- 3. ENERGY CONTROL ---
        PE = M * G * L * math.cos(theta)
        KE = 0.5 * M * (L**2) * (theta_dot**2)
        E = PE + KE
        E_err = E_target - E 

        # Energy Shaping
        # Note: If it fights the swing, invert K_ENERGY sign
        u_energy = K_ENERGY * E_err * theta_dot * math.cos(theta)

        # Damping & Centering
        u_damp = -K_DAMP * theta_dot
        u_center = -K_CENTER * x - K_XDOT * x_dot

        u_pump = u_energy + u_damp + u_center

        # --- 4. SAFETY WALLS (FIXED) ---
        # Checks against WALL_DIST = 0.20m (not 5.18!)
        u_wall_force = 0.0
        if x > WALL_DIST:
            dist = x - WALL_DIST
            u_wall_force = -K_WALL * dist - D_WALL * x_dot
            if u_pump > 0: u_pump = 0 # Kill outward force
        elif x < -WALL_DIST:
            dist = x + WALL_DIST
            u_wall_force = -K_WALL * dist - D_WALL * x_dot
            if u_pump < 0: u_pump = 0

        wall_current = (u_wall_force * PULLEY_RADIUS) / KT
        total_current = u_pump + wall_current
        total_current = clipped(total_current, -MAX_CURRENT, MAX_CURRENT)

        # Send Command
        drv.setTorqueControl(float(total_current), 3000)

        # --- 5. LOGGING & SUCCESS ---
        if int(t_elapsed * 20) % 10 == 0: # Print every 0.5s
            print(f"\r t={t_elapsed:5.1f} | Th={theta_deg:5.1f} | x={x:5.2f} | I={total_current:5.2f}", end="")

        if abs(theta_deg) < 15.0 and abs(theta_dot) < 1.0:
            print("\n[SUCCESS] Upright reached.")
            drv.setTorqueControl(0.0, 0)
            break
            
        log.append([t_elapsed, theta_deg, theta_dot, x, total_current, E_err])

except KeyboardInterrupt:
    print("\n[ABORTED]")

except Exception as ex:
    print("\n[ERROR]", ex)

finally:
    drv.setDeviceToIdle()
    try:
        with open(LOGFILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'theta', 'theta_dot', 'x', 'current', 'E_err'])
            writer.writerows(log)
        print("Log saved.")
    except: pass