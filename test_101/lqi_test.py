from nmotion_transport import USBInterface, Driver, NCoder
import time
import numpy as np
import sys

# ==========================
#      CONFIGURATION
# ==========================
PULLEY_RADIUS  = 0.0075    # 7.5 mm
UPRIGHT_OFFSET = 146.09    # Vertical Angle
KT             = 0.0306    # Torque Constant
MAX_CURRENT    = 8.0       # Current Limit

# --- SIGNAL PROCESSING ---
ALPHA     = 0.15           # Filter (Keep your working value)
LOOP_FREQ = 1000            # Hz (Target Frequency)
DT_TARGET = 1.0 / LOOP_FREQ

# --- LQI GAINS ---
# Standard LQR (Your working set)
K  = np.array([ -10.0,  -11.83,  67.34,  7.5 ])

# Integral Gain (LQI)
# Corrects steady-state drift (cable drag, friction)
# Start small (-1.0 to -5.0)
Ki = -1.0

# ==========================
#      GLOBAL SETUP
# ==========================
try:
    iface = USBInterface("COM3")
    drv0  = Driver(1, iface)
    ncoder = NCoder(2, iface)
except Exception as e:
    print(f"HARDWARE ERROR: {e}")
    sys.exit(1)

v_pos = 13
hey   = True

# State Storage
avg_x_dot = 0.0
avg_theta_dot = 0.0
integral_error = 0.0  # Accumulator for LQI

# ==========================
#    HELPER FUNCTIONS
# ==========================
def get_state(start_rot, prev_state, dt):
    global avg_x_dot, avg_theta_dot
    
    s1, raw_angle = ncoder.getAbsoluteAngle()
    s2, raw_rot   = drv0.getMotorPosition()
    if s1 != 0 or s2 != 0: return None 

    # Angle
    theta_deg = raw_angle - UPRIGHT_OFFSET
    while theta_deg > 180:  theta_deg -= 360
    while theta_deg < -180: theta_deg += 360
    theta_rad = np.deg2rad(theta_deg)

    # Position
    x_meters = (raw_rot - start_rot) * (2 * np.pi * PULLEY_RADIUS)

    # Filtered Velocity
    if dt > 0:
        raw_x_dot     = (x_meters - prev_state[0]) / dt
        raw_theta_dot = (theta_rad - prev_state[2]) / dt
        
        avg_x_dot     = (ALPHA * raw_x_dot) + ((1 - ALPHA) * avg_x_dot)
        avg_theta_dot = (ALPHA * raw_theta_dot) + ((1 - ALPHA) * avg_theta_dot)
    else:
        avg_x_dot = 0.0; avg_theta_dot = 0.0

    return np.array([x_meters, avg_x_dot, theta_rad, avg_theta_dot]), theta_deg

# ==========================
#    MAIN CONTROL LOOP
# ==========================
print(f"--- LQI CONTROLLER ({LOOP_FREQ} Hz) ---")

if __name__ == "__main__":
    state = np.zeros(4)
    start_rot = None 
    
    # High-Res Timer
    prev_time = time.perf_counter()
    
    try:
        # 1. Zeroing
        print("[INIT] Zeroing...", end="")
        while start_rot is None and hey:
            stat, val = drv0.getMotorPosition()
            if stat == 0:
                start_rot = val
                print(f" OK: {start_rot:.2f}")
            else:
                time.sleep(0.1)

        # 2. Start
        print("[READY] Hold Vertical. 2s...")
        time.sleep(2)
        print(">>> ENGAGED <<<")
        
        prev_time = time.perf_counter()

        while hey:
            loop_start = time.perf_counter()

            # A. Timing Calculation
            now = time.perf_counter()
            dt = now - prev_time
            if dt <= 0: continue

            # B. Get State
            data = get_state(start_rot, state, dt)
            if data is None: continue 
            new_state, theta_deg = data

            # C. Safety (Rail)
            curr_rot = (new_state[0] / (2 * np.pi * PULLEY_RADIUS))
            if abs(curr_rot) > v_pos:
                print(f"[STOP] Rail Limit")
                hey = False; break

            # --- D. LQI CALCULATION (The Upgrade) ---
            
            # 1. Accumulate Error (Distance from 0)
            integral_error += new_state[0] * dt
            
            # 2. Anti-Windup (Prevent it from growing infinite)
            # Clamps the "memory" to equivalent of +/- 0.5 meters error
            integral_error = np.clip(integral_error, -0.5, 0.5)

            # 3. Calculate Forces
            force_lqr = -np.dot(K, new_state)    # Standard Balance
            force_int = -(Ki * integral_error)   # Centering Force
            
            total_force = force_lqr + force_int

            # E. Command
            current_cmd = (total_force * PULLEY_RADIUS) / KT
            current_cmd = np.clip(current_cmd, -MAX_CURRENT, MAX_CURRENT)
            drv0.setTorqueControl(current_cmd, 3000)

            # F. Update
            state = new_state
            prev_time = now
            
            # --- G. FREQUENCY LOCK ---
            # Wait until exactly 0.005s (200Hz) has passed since loop_start
            while (time.perf_counter() - loop_start) < DT_TARGET:
                pass

    except (KeyboardInterrupt, Exception) as e:
        print(f"\n[EXIT] {e}")
    
    finally:
        try: drv0.setDeviceToIdle() 
        except: pass
        print("[DONE]")