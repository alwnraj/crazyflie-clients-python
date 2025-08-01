import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time
from cflib.positioning.motion_commander import MotionCommander

# Initialize drivers
cflib.crtp.init_drivers(enable_debug_driver=False)

# Scan for Crazyflies
print("Scanning for Crazyflies...")
available = cflib.crtp.scan_interfaces()

if not available:
    print("No Crazyflies found!")
    exit(1)

print(f"Found {len(available)} Crazyflie(s):")
for uri in available:
    print(f"  {uri}")

# Try to connect to the first one
uri = available[0][0]
print(f"\nTrying to connect to {uri}...")

try:
    with SyncCrazyflie(uri) as scf:
        print("✓ Successfully connected!")
        print("✓ Communication established")
        
        # Try to read a parameter
        try:
            version = scf.cf.param.get_value("firmware.revision0")
            print(f"✓ Firmware revision: {version}")
        except:
            print("⚠ Could not read firmware version")
            
        time.sleep(2)
        print("✓ Connection test completed successfully!")
        
        with MotionCommander(scf) as mc:
            # The Flow v2 deck enables precise position control
            mc.forward(1.0)  # Move forward 1 meter
            mc.left(0.5)     # Move left 0.5 meters
            mc.up(0.3)       # Move up 0.3 meters
        
except Exception as e:
    print(f"✗ Connection failed: {e}")
    print("\nTroubleshooting:")
    print("1. Check Crazyflie battery")
    print("2. Move closer to Crazyflie")
    print("3. Try different USB port")
    print("4. Check for interference") 