
from nmotion_transport import USBInterface, Driver, NCoder
import time
import numpy as np

# ==========================
#      CONFIGURATION
# ==========================
# Hardware
PULLEY_RADIUS  = 0.0075    # 7.5 mm
UPRIGHT_OFFSET = 146.206    # Encoder angle when vertical
KT             = 0.0306    # Torque Constant
MAX_CURRENT    = 8.0       # Current Limit (Amps)

# --- FILTER CONFIGURATION ---
# 0.15 = Very Smooth (No vibration, slightly slower)
# 0.30 = Reactive (Snappier, slight buzz)
ALPHA = 0.18

# --- LQR GAINS ---
# Current Best Set

K = np.array([-10.,-11.77490462,50.73172986, 6.3])

# ==========================
#      GLOBAL SETUP
# ==========================
iface = USBInterface("COM3")
drv0  = Driver(1, iface)
ncoder = NCoder(2, iface)

v_pos  = 13        # Rail Limit (+/- 13 rotations)
hey    = True      # Loop Flag

# Filter Storage (Global state for smoothing)
avg_x_dot = 0.0
avg_theta_dot = 0.0

# ==========================
#    HELPER FUNCTIONS
# ==========================

def get_state(start_rot, prev_state, dt):
    """ 
    Calculates State Vector [x, dx, theta, dtheta] 
    IMPROVEMENT: Added Low Pass Filter to Velocities
    """
    global avg_x_dot, avg_theta_dot
    
    # 1. Read Sensors
    s1, raw_angle = ncoder.getAbsoluteAngle()
    s2, raw_rot   = drv0.getMotorPosition()
    
    if s1 != 0 or s2 != 0: return None # Comm Error

    # 2. Angle Processing
    theta_deg = raw_angle - UPRIGHT_OFFSET
    # Wrap to -180...180 range
    while theta_deg > 180:  theta_deg -= 360
    while theta_deg < -180: theta_deg += 360
    theta_rad = np.deg2rad(theta_deg)

    # 3. Position Processing (Meters)
    x_meters = (raw_rot - start_rot) * (2 * np.pi * PULLEY_RADIUS)

    # 4. Velocities with LOW PASS FILTER
    if dt > 0:
        # Raw instantaneous calculation
        raw_x_dot     = (x_meters - prev_state[0]) / dt
        raw_theta_dot = (theta_rad - prev_state[2]) / dt
        
        # Apply Exponential Moving Average (EMA) Filter
        # Smooths out the "steps" from the encoder
        avg_x_dot     = (ALPHA * raw_x_dot) + ((1 - ALPHA) * avg_x_dot)
        avg_theta_dot = (ALPHA * raw_theta_dot) + ((1 - ALPHA) * avg_theta_dot)
    else:
        avg_x_dot = 0
        avg_theta_dot = 0

    return np.array([x_meters, avg_x_dot, theta_rad, avg_theta_dot]), theta_deg

# ==========================
#    MAIN CONTROL LOOP
# ==========================
print("Starting IMPROVED LQR control...")

if __name__ == "__main__":
    # Init State
    state = np.zeros(4)
    start_rot = None 
    
    # Safety Counters (Prevents false triggers)
    fall_counter = 0 
    
    # IMPROVEMENT: Use High Resolution Timer
    prev_time = time.perf_counter()
    
    try:
        # --- 1. ZEROING PHASE ---
        print("Reading Initial Position...")
        while start_rot is None and hey:
            stat, val = drv0.getMotorPosition()
            if stat == 0:
                start_rot = val
                print(f"Zeroed at: {start_rot}")
            time.sleep(0.1)

        # --- 2. STARTUP PHASE ---
        print("Hold Vertical... (Starting in 2s)")
        time.sleep(2)
        print("*** ENGAGED ***")
        
        # Reset timer right before loop starts
        prev_time = time.perf_counter()

        # --- 3. CONTROL LOOP ---
        while hey:
            # High Precision Timing
            now = time.perf_counter()
            dt = now - prev_time
            if dt <= 0.0001: continue

            # A. Get State (Now Filtered)
            data = get_state(start_rot, state, dt)
            if data is None: continue # Skip bad frames
            
            new_state, theta_deg = data

            # B. SAFETY CHECKS (Integrated into main loop for stability)
            
            # 1. Rail Limits
            # We use new_state[0] (meters) instead of raw rotations for cleaner math
            # But relying on raw rotations matches your original logic:
            current_rot = (new_state[0] / (2 * np.pi * PULLEY_RADIUS)) + start_rot
            if not (-v_pos < (current_rot - start_rot) < v_pos):
                print(f"LIMIT REACHED: {current_rot - start_rot:.2f}")
                hey = False
                break
                
            # 2. Fall Detection (Debounced)
            # Only kill if angle is bad for 5 consecutive frames
            if abs(theta_deg) > 30.0: # Widen to 30 deg for aggressive catching
                fall_counter += 1
            else:
                fall_counter = 0
                
            # if fall_counter > 5:
            #     print(f"FALL DETECTED: {theta_deg:.1f}")
            #     hey = False
            #     break

            # C. LQR Calculation
            force = -np.dot(K, new_state)

            # D. Command Motor
            current_cmd = (force * PULLEY_RADIUS) / KT
            current_cmd = np.clip(current_cmd, -MAX_CURRENT, MAX_CURRENT)
            drv0.setTorqueControl(current_cmd, 3000)

            # E. Update History
            state = new_state
            prev_time = now

    except (KeyboardInterrupt, Exception) as e:
        print("\nError/Stop:", e)
        hey = False
    
    finally:
        drv0.setDeviceToIdle()
        print("Stopped.")
        