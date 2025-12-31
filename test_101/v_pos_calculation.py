from nmotion_transport import USBInterface, Driver
import time
import sys

# ==========================
#          SETUP
# ==========================
print("Connecting to USB...")
try:
    iface = USBInterface("COM3")
    drv0  = Driver(1, iface)
    print("USB Opened.")
except Exception as e:
    print(f"FAILED to open USB: {e}")
    sys.exit()

def get_reading_debug():
    """Tries to read for 3 seconds, printing errors if they happen."""
    print("   [Debug] Talking to motor...", end="", flush=True)
    
    for i in range(30): # Try for 3 seconds (30 * 0.1s)
        try:
            stat, val = drv0.getMotorPosition()
            if stat == 0:
                print(" OK!")
                return val
            else:
                # Motor is connected but returning error codes
                print(f".(Err:{stat})", end="", flush=True)
        except Exception as e:
            print(f".(Exc:{e})", end="", flush=True)
            
        time.sleep(0.1)
        
    print("\n   [ERROR] Connection Timed Out. Motor not responding.")
    return None

def main():
    print("\n--- v_pos CALCULATOR (DEBUG MODE) ---")
    
    # 1. Disable Motor
    print("Setting Idle Mode...")
    drv0.setDeviceToIdle()
    
    # Test read immediately to verify comms
    test_val = get_reading_debug()
    if test_val is None:
        print("CRITICAL: Cannot talk to motor. Check Cables/Power Switch.")
        return

    print("\nMotor is IDLE. You can move it by hand.")
    
    # 2. Set Center
    # We use a loop here to flush any 'stuck' inputs
    while True:
        try:
            input("\n[STEP 1] Move Cart to CENTER. Press ENTER to read...")
            break
        except SyntaxError:
            pass 
            
    center_pos = get_reading_debug()
    if center_pos is None: return
    print(f" -> Center stored: {center_pos:.4f}")
    
    # 3. Set Limit
    while True:
        try:
            input("\n[STEP 2] Move Cart to LIMIT. Press ENTER to read...")
            break
        except SyntaxError:
            pass

    limit_pos = get_reading_debug()
    if limit_pos is None: return
    print(f" -> Limit stored: {limit_pos:.4f}")
    
    # 4. Calculate
    raw_distance = abs(limit_pos - center_pos)
    safe_v_pos = raw_distance - 0.5
    
    if safe_v_pos < 1.0:
        print("\n[WARNING] Calculated distance is very small. Did you move the cart?")
    
    print("\n" + "="*30)
    print(f"Raw Distance:     {raw_distance:.4f} rotations")
    print(f"Safety Buffer:    0.5000 rotations")
    print("-" * 30)
    print(f"RECOMMENDED v_pos = {safe_v_pos:.2f}")
    print("="*30)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled.")
    finally:
        try:
            drv0.setDeviceToIdle()
        except:
            pass