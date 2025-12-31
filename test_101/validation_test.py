from nmotion_transport import USBInterface, Driver, NCoder
import time
import numpy as np
import sys

# ==========================================
#              CONFIGURATION
# ==========================================

# --- HARDWARE ---
PULLEY_RADIUS  = 0.0075    # 7.5 mm
UPRIGHT_OFFSET = 146.09     # Encoder angle when perfectly vertical

# --- GAINS TO TEST ---
# [ Pos(x),  Vel(dx),  Angle(th),  AngVel(dth) ]
# Standard Logic: Pos (+) to restore center, Angle (-) to catch fall
K = np.array([ -10.0,  -11.83,  67.34,  6.8 ])

# ==========================================
#              TEST SCRIPT
# ==========================================

try:
    iface = USBInterface("COM3")
    drv0  = Driver(1, iface)
    ncoder = NCoder(2, iface)
except Exception as e:
    print(f"Hardware Error: {e}")
    sys.exit()

def get_continuous_angle(raw_angle, offset):
    """Normalize raw angle to -180 to +180 centered on offset"""
    angle = raw_angle - offset
    while angle > 180:  angle -= 360
    while angle < -180: angle += 360
    return angle # Degrees

def main():
    print("--- LQR GAIN CHECKER (MOTOR IDLE) ---")
    print("1. Tilt Rod RIGHT (+Angle) -> Should say 'PUSH RIGHT' (Catch)")
    print("2. Move Cart RIGHT (+Pos)  -> Should say 'PUSH LEFT'  (Center)")
    print("-------------------------------------------------------------")
    time.sleep(2)

    # Disable Motor for Safety
    drv0.setDeviceToIdle() 
    
    # Zeroing
    stat, start_pos_rot = drv0.getMotorPosition()
    if stat != 0:
        print("Error: Motor Comm Failed")
        return

    # State tracking
    prev_time = time.time()
    state = np.zeros(4) # [x, dx, theta, dtheta]

    try:
        while True:
            # Slow loop for readability
            time.sleep(0.05) 
            
            now = time.time()
            dt = now - prev_time
            if dt <= 0.0001: continue
            
            # 1. Read Sensors
            s1, raw_angle = ncoder.getAbsoluteAngle()
            s2, raw_rot   = drv0.getMotorPosition()
            
            if s1 != 0 or s2 != 0: continue

            # 2. Calculate State
            # Position (Meters)
            x_meters = (raw_rot - start_pos_rot) * (2 * np.pi * PULLEY_RADIUS)
            
            # Angle (Radians)
            theta_deg = get_continuous_angle(raw_angle, UPRIGHT_OFFSET)
            theta_rad = np.deg2rad(theta_deg)

            # Velocities
            x_dot     = (x_meters - state[0]) / dt
            theta_dot = (theta_rad - state[2]) / dt

            # Update State
            state = np.array([x_meters, x_dot, theta_rad, theta_dot])
            prev_time = now

            # 3. Calculate LQR Output
            # u = -K * x
            u_force = -np.dot(K, state)

            # 4. Visual Feedback
            pos_label = "RIGHT" if x_meters > 0 else "LEFT "
            ang_label = "RIGHT" if theta_deg > 0 else "LEFT "
            
            # Logic Check
            cmd_label = ""
            if u_force > 0.5:
                cmd_label = ">> PUSH RIGHT >> (Catching/Pushing)"
            elif u_force < -0.5:
                cmd_label = "<< PUSH LEFT  << (Catching/Pushing)"
            else:
                cmd_label = "--  NEUTRAL   --"

            # Dynamic Print
            print(f"\r Pos: {x_meters:.3f}m [{pos_label}] | Tilt: {theta_deg:.1f}° [{ang_label}] || CMD: {u_force:.2f}N  {cmd_label}", end="")

    except KeyboardInterrupt:
        print("\nTest Complete.")

if __name__ == "__main__":
    main()